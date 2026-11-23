// Copyright 2025, rcelyte
// Copyright 2026, Sapphire
// SPDX-License-Identifier: BSL-1.0

#include "solarxr_interface.h"
#include "feeder.h"
#include "protocol.h"
#include "solarxr_ipc_message.h"
#include "solarxr_ipc_socket.h"

#include "math/m_api.h"
#include "math/m_relation_history.h"
#include "math/m_vec3.h"
#include "os/os_threading.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_time.h"
#include "util/u_trace_marker.h"

#include <endian.h>
#include <stdatomic.h>
#include <stdio.h>
#include <wchar.h>

#define MAX_GENERIC_TRACKERS 32

DEBUG_GET_ONCE_LOG_OPTION(solarxr_log, "SOLARXR_LOG", U_LOGGING_INFO)

struct solarxr_device;
struct solarxr_generic_tracker
{
	struct xrt_device base;
	struct os_mutex mutex;
	struct solarxr_device *parent; // weak reference
	uint32_t index;
	struct m_relation_history *history; // weak reference
	enum solarxr_body_part role;
	float charge; // -1.f == no battery info
	bool charging;
};

struct solarxr_device
{
	struct xrt_device base;
	struct feeder feeder;
	struct os_thread thread;
	struct solarxr_ipc_socket socket;
	struct os_mutex mutex;
	bool shook_hands;
	_Atomic(timepoint_ns) timestamp;
	_Atomic(uint64_t) enabled_bones;
	static_assert(SOLARXR_BODY_PART_MAX_ENUM <= 64, "bitfield too small");
	struct solarxr_device_bone
	{
		struct xrt_pose pose;
		float length;
	} bones[SOLARXR_BODY_PART_MAX_ENUM];
	struct m_relation_history *trackers[MAX_GENERIC_TRACKERS];
	struct solarxr_generic_tracker *tracker_refs[MAX_GENERIC_TRACKERS];
	bool use_trackers;
	uint32_t generation;
	struct xrt_tracking_origin standalone_origin;
};

struct span
{
	size_t length;
	const uint8_t *data;
};

static void
solarxr_device_destroy(struct xrt_device *xdev);
static void
solarxr_generic_tracker_destroy(struct xrt_device *xdev);

static inline struct solarxr_device *
solarxr_device(struct xrt_device *const xdev)
{
	if (xdev == NULL || xdev->destroy != solarxr_device_destroy) {
		return NULL;
	}
	return (struct solarxr_device *)xdev;
}

static inline struct solarxr_generic_tracker *
solarxr_generic_tracker(struct xrt_device *const xdev)
{
	if (xdev == NULL || xdev->destroy != solarxr_generic_tracker_destroy) {
		return NULL;
	}
	return (struct solarxr_generic_tracker *)xdev;
}

static xrt_result_t
solarxr_device_update_inputs(struct xrt_device *const xdev)
{
	struct solarxr_device *const device = solarxr_device(xdev);
	assert(device != NULL);

	for (uint32_t i = 0; i < device->base.input_count; ++i) {
		device->base.inputs[i].timestamp = atomic_load(&device->timestamp);
	}
	return XRT_SUCCESS;
}

static xrt_result_t
solarxr_generic_tracker_update_inputs(struct xrt_device *const xdev)
{
	struct solarxr_generic_tracker *const device = solarxr_generic_tracker(xdev);
	assert(device != NULL);
	os_mutex_lock(&device->mutex);

	struct solarxr_device *const parent = device->parent;
	if (parent == NULL) {
		device->base.inputs[0].active = false;
	} else {
		device->base.inputs[0].active = (atomic_load(&parent->enabled_bones) & (1llu << device->role)) != 0;
		device->base.inputs[0].timestamp = atomic_load(&parent->timestamp);
	}

	os_mutex_unlock(&device->mutex);
	return XRT_SUCCESS;
}

static inline struct xrt_body_skeleton_joint_fb
offset_joint(const struct xrt_body_skeleton_joint_fb parent, const int32_t name, const struct xrt_vec3 offset)
{
	return (struct xrt_body_skeleton_joint_fb){
	    .pose =
	        {
	            .orientation = parent.pose.orientation,
	            .position = m_vec3_add(parent.pose.position, offset),
	        },
	    .joint = name,
	    .parent_joint = parent.joint,
	};
}

// TODO: filter enabled bones
static xrt_result_t
solarxr_device_get_body_skeleton(struct xrt_device *const xdev,
                                 const enum xrt_input_name body_tracking_type,
                                 struct xrt_body_skeleton *const out_value)
{
	struct xrt_body_skeleton_joint_fb *joints;
	uint32_t joint_count;
	int32_t none;
	switch (body_tracking_type) {
	case XRT_INPUT_FB_BODY_TRACKING: {
		joints = out_value->body_skeleton_fb.joints;
		joint_count = ARRAY_SIZE(out_value->body_skeleton_fb.joints);
		none = XRT_BODY_JOINT_NONE_FB;
		break;
	}
	case XRT_INPUT_META_FULL_BODY_TRACKING: {
		joints = out_value->full_body_skeleton_meta.joints;
		joint_count = ARRAY_SIZE(out_value->full_body_skeleton_meta.joints);
		none = XRT_FULL_BODY_JOINT_NONE_META;
		break;
	}
	default: return XRT_ERROR_NOT_IMPLEMENTED;
	}

	struct solarxr_device *const device = solarxr_device(xdev);
	assert(device != NULL);
	for (uint32_t i = 0; i < joint_count; ++i) {
		joints[i] = (struct xrt_body_skeleton_joint_fb){XRT_POSE_IDENTITY, none, none};
	}

	// The standard doesn't describe the layout for these joints more specifically than being "a T-pose"
	// clang-format off
	joints[0] = (struct xrt_body_skeleton_joint_fb){XRT_POSE_IDENTITY, XRT_BODY_JOINT_HEAD_FB, XRT_BODY_JOINT_ROOT_FB};
	joints[1] = offset_joint(joints[0], XRT_BODY_JOINT_NECK_FB, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_NECK].length, 0.f});
	joints[2] = offset_joint(joints[1], XRT_BODY_JOINT_CHEST_FB, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_UPPER_CHEST].length, 0.f});
	joints[3] = offset_joint(joints[2], XRT_BODY_JOINT_SPINE_UPPER_FB, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_LOWER_CHEST].length, 0.f});
	joints[4] = offset_joint(joints[3], XRT_BODY_JOINT_SPINE_MIDDLE_FB, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_UPPER_WAIST].length, 0.f});
	joints[5] = offset_joint(joints[4], XRT_BODY_JOINT_SPINE_LOWER_FB, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_LOWER_WAIST].length, 0.f});
	joints[6] = offset_joint(joints[5], XRT_BODY_JOINT_HIPS_FB, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_HIP].length, 0.f});
	joints[7] = offset_joint(joints[1], XRT_BODY_JOINT_LEFT_SHOULDER_FB, (struct xrt_vec3){-device->bones[SOLARXR_BODY_PART_LEFT_SHOULDER].length, 0.f, 0.f});
	joints[8] = offset_joint(joints[1], XRT_BODY_JOINT_RIGHT_SHOULDER_FB, (struct xrt_vec3){device->bones[SOLARXR_BODY_PART_RIGHT_SHOULDER].length, 0.f, 0.f});
	joints[9] = offset_joint(joints[7], XRT_BODY_JOINT_LEFT_ARM_UPPER_FB, (struct xrt_vec3){-device->bones[SOLARXR_BODY_PART_LEFT_UPPER_ARM].length, 0.f, 0.f});
	joints[10] = offset_joint(joints[8], XRT_BODY_JOINT_RIGHT_ARM_UPPER_FB, (struct xrt_vec3){device->bones[SOLARXR_BODY_PART_RIGHT_UPPER_ARM].length, 0.f, 0.f});
	joints[11] = offset_joint(joints[9], XRT_BODY_JOINT_LEFT_ARM_LOWER_FB, (struct xrt_vec3){-device->bones[SOLARXR_BODY_PART_LEFT_LOWER_ARM].length, 0.f, 0.f});
	joints[12] = offset_joint(joints[10], XRT_BODY_JOINT_RIGHT_ARM_LOWER_FB, (struct xrt_vec3){device->bones[SOLARXR_BODY_PART_RIGHT_LOWER_ARM].length, 0.f, 0.f});
	joints[13] = offset_joint(joints[11], XRT_BODY_JOINT_LEFT_HAND_WRIST_FB, (struct xrt_vec3){-device->bones[SOLARXR_BODY_PART_LEFT_HAND].length, 0.f, 0.f});
	joints[14] = offset_joint(joints[12], XRT_BODY_JOINT_RIGHT_HAND_WRIST_FB, (struct xrt_vec3){device->bones[SOLARXR_BODY_PART_RIGHT_HAND].length, 0.f, 0.f});
	if (body_tracking_type != XRT_INPUT_META_FULL_BODY_TRACKING) {
		return XRT_SUCCESS;
	}
	joints[15] = offset_joint(joints[5], XRT_FULL_BODY_JOINT_LEFT_UPPER_LEG_META, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_LEFT_UPPER_LEG].length, 0.f});
	joints[16] = offset_joint(joints[5], XRT_FULL_BODY_JOINT_RIGHT_UPPER_LEG_META, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_RIGHT_UPPER_LEG].length, 0.f});
	joints[17] = offset_joint(joints[15], XRT_FULL_BODY_JOINT_LEFT_LOWER_LEG_META, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_LEFT_LOWER_LEG].length, 0.f});
	joints[18] = offset_joint(joints[16], XRT_FULL_BODY_JOINT_RIGHT_LOWER_LEG_META, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_RIGHT_LOWER_LEG].length, 0.f});
	joints[19] = offset_joint(joints[17], XRT_FULL_BODY_JOINT_LEFT_FOOT_TRANSVERSE_META, (struct xrt_vec3){0.f, 0.f, -device->bones[SOLARXR_BODY_PART_LEFT_FOOT].length});
	joints[20] = offset_joint(joints[18], XRT_FULL_BODY_JOINT_RIGHT_FOOT_TRANSVERSE_META, (struct xrt_vec3){0.f, 0.f, -device->bones[SOLARXR_BODY_PART_RIGHT_FOOT].length});
	// clang-format on
	return XRT_SUCCESS;
}

// TODO: filter enabled bones
static xrt_result_t
solarxr_device_get_body_joints(struct xrt_device *const xdev,
                               const enum xrt_input_name body_tracking_type,
                               const int64_t desired_timestamp_ns,
                               struct xrt_body_joint_set *const out_value)
{
	static const uint32_t jointMap[SOLARXR_BODY_PART_MAX_ENUM] = {
	    [SOLARXR_BODY_PART_HEAD] = XRT_BODY_JOINT_HEAD_FB,
	    [SOLARXR_BODY_PART_NECK] = XRT_BODY_JOINT_NECK_FB,
	    [SOLARXR_BODY_PART_UPPER_CHEST] = XRT_BODY_JOINT_CHEST_FB,
	    [SOLARXR_BODY_PART_LOWER_CHEST] = XRT_BODY_JOINT_SPINE_UPPER_FB,
	    [SOLARXR_BODY_PART_UPPER_WAIST] = XRT_BODY_JOINT_SPINE_MIDDLE_FB,
	    [SOLARXR_BODY_PART_LOWER_WAIST] = XRT_BODY_JOINT_SPINE_LOWER_FB,
	    [SOLARXR_BODY_PART_HIP] = XRT_BODY_JOINT_HIPS_FB,
	    [SOLARXR_BODY_PART_LEFT_UPPER_LEG] = XRT_FULL_BODY_JOINT_LEFT_UPPER_LEG_META,
	    [SOLARXR_BODY_PART_RIGHT_UPPER_LEG] = XRT_FULL_BODY_JOINT_RIGHT_UPPER_LEG_META,
	    [SOLARXR_BODY_PART_LEFT_LOWER_LEG] = XRT_FULL_BODY_JOINT_LEFT_LOWER_LEG_META,
	    [SOLARXR_BODY_PART_RIGHT_LOWER_LEG] = XRT_FULL_BODY_JOINT_RIGHT_LOWER_LEG_META,
	    [SOLARXR_BODY_PART_LEFT_FOOT] = XRT_FULL_BODY_JOINT_LEFT_FOOT_TRANSVERSE_META,
	    [SOLARXR_BODY_PART_RIGHT_FOOT] = XRT_FULL_BODY_JOINT_RIGHT_FOOT_TRANSVERSE_META,
	    [SOLARXR_BODY_PART_LEFT_SHOULDER] = XRT_BODY_JOINT_LEFT_SHOULDER_FB,
	    [SOLARXR_BODY_PART_RIGHT_SHOULDER] = XRT_BODY_JOINT_RIGHT_SHOULDER_FB,
	    [SOLARXR_BODY_PART_LEFT_UPPER_ARM] = XRT_BODY_JOINT_LEFT_ARM_UPPER_FB,
	    [SOLARXR_BODY_PART_RIGHT_UPPER_ARM] = XRT_BODY_JOINT_RIGHT_ARM_UPPER_FB,
	    [SOLARXR_BODY_PART_LEFT_LOWER_ARM] = XRT_BODY_JOINT_LEFT_ARM_LOWER_FB,
	    [SOLARXR_BODY_PART_RIGHT_LOWER_ARM] = XRT_BODY_JOINT_RIGHT_ARM_LOWER_FB,
	    [SOLARXR_BODY_PART_LEFT_HAND] = XRT_BODY_JOINT_LEFT_HAND_WRIST_FB,
	    [SOLARXR_BODY_PART_RIGHT_HAND] = XRT_BODY_JOINT_RIGHT_HAND_WRIST_FB,
	};
	struct xrt_body_joint_location_fb *joints;
	uint32_t joint_count;
	switch (body_tracking_type) {
	case XRT_INPUT_FB_BODY_TRACKING: {
		joints = out_value->body_joint_set_fb.joint_locations;
		joint_count = ARRAY_SIZE(out_value->body_joint_set_fb.joint_locations);
		break;
	}
	case XRT_INPUT_META_FULL_BODY_TRACKING: {
		joints = out_value->full_body_joint_set_meta.joint_locations;
		joint_count = ARRAY_SIZE(out_value->full_body_joint_set_meta.joint_locations);
		break;
	}
	default: return XRT_ERROR_NOT_IMPLEMENTED;
	}

	struct solarxr_device *const device = solarxr_device(xdev);
	assert(device != NULL);

	os_mutex_lock(&device->mutex);

	out_value->base_body_joint_set_meta.sample_time_ns =
	    atomic_load_explicit(&device->timestamp, memory_order_relaxed);
	out_value->base_body_joint_set_meta.confidence = 1.f; // N/A
	out_value->base_body_joint_set_meta.skeleton_changed_count = device->generation;
	out_value->base_body_joint_set_meta.is_active = device->shook_hands;

	for (uint32_t i = 0; i < joint_count; ++i) {
		joints[i].relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
	}

	if (!device->shook_hands) {
		goto unlock;
	}

	for (enum solarxr_body_part part = 0; part < ARRAY_SIZE(device->bones); ++part) {
		const struct xrt_pose pose = device->bones[part].pose;
		static_assert(ARRAY_SIZE(jointMap) == ARRAY_SIZE(device->bones), "");
		const uint32_t index = jointMap[part];
		if (index == 0 || index >= joint_count ||
		    memcmp(&pose.orientation, &(struct xrt_quat){0}, sizeof(struct xrt_quat)) == 0) {
			continue;
		}

		joints[index].relation = (struct xrt_space_relation){
		    .relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
		                      XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
		                      XRT_SPACE_RELATION_POSITION_TRACKED_BIT,
		    .pose = pose,
		};
	}

	out_value->body_pose = (struct xrt_space_relation){
	    .relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                      XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT,
	    .pose = XRT_POSE_IDENTITY,
	};

unlock:
	os_mutex_unlock(&device->mutex);
	return XRT_SUCCESS;
}

static void
solarxr_device_handle_bones(struct solarxr_device *const device,
                            const struct solarxr_bone bones[ARRAY_SIZE(device->bones)])
{
	XRT_TRACE_MARKER();

	os_mutex_lock(&device->mutex);

	XRT_TRACE_BEGIN(solarxr_device_handle_generic_trackers);
	for (size_t i = 0; i < ARRAY_SIZE(device->tracker_refs); ++i) {
		const struct solarxr_generic_tracker *const tracker = device->tracker_refs[i];
		if (tracker == NULL || tracker->role == SOLARXR_BODY_PART_NONE) {
			break;
		}

		struct m_relation_history *const history = device->trackers[i];
		if (history == NULL) {
			continue;
		}

		const struct solarxr_bone *const bone = &bones[tracker->role];

		const struct xrt_space_relation relation = {
		    .relation_flags =
		        XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
		        XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
		        XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT,
		    .pose =
		        (struct xrt_pose){
		            .orientation = bone->orientation,
		            .position = bone->tail_position,
		        },
		    .linear_velocity = bone->linear_velocity,
		    .angular_velocity = bone->angular_velocity,
		};
		m_relation_history_push(history, &relation, device->socket.timestamp);
	}
	XRT_TRACE_END(solarxr_device_handle_generic_trackers);

	bool generation_increased = false;
	for (uint32_t i = 0; i < ARRAY_SIZE(device->bones); ++i) {
		const struct solarxr_bone *const bone = &bones[i];
		if (!generation_increased &&
		    memcmp(&bone->bone_length, &device->bones[i].length, sizeof(bone->bone_length)) != 0) {
			++device->generation;
			generation_increased = true;
		}

		device->bones[i].pose = (struct xrt_pose){
		    .orientation = bone->rotation,
		    .position = bone->head_position,
		};
		device->bones[i].length = bone->bone_length;
	}

	os_mutex_unlock(&device->mutex);
}

static void *
solarxr_network_thread(void *const ptr)
{
	struct solarxr_device *const device = (struct solarxr_device *)ptr;

	const char *const thread_name = "SolarXR Network";

	U_TRACE_SET_THREAD_NAME(thread_name);
	os_thread_name_self(thread_name);

	static const char driver_name[] = "Monado";
	struct
	{
		uint8_t head[sizeof(struct solarxr_ipc_message)];
		struct handshake_request_packet
		{
			uint32_t _root;
			uint16_t _table_bundle[5];
			struct
			{ // table MessageBundle
				int32_t _table;
				uint32_t driver_msgs; // vector*
			} bundle;
			struct
			{ // vector<table DriverMessageHeader>
				uint32_t length;
				uint32_t values[1]; // table*
			} driver_msgs;
			uint16_t _table_driver_msgs_0[6];
			struct
			{ // table DriverMessageHeader
				int32_t _table;
				uint32_t message;     // table*
				uint8_t message_type; // enum DriverMessage
				uint8_t _pad[3];
			} driver_msgs_0;
			uint16_t _table_request[4];
			struct
			{ // table HandshakeRequest
				int32_t _table;
				uint32_t driver_name; // vector*
				uint32_t bone_mask;   // table*
			} request;
			struct
			{ // vector<char>
				uint32_t length;
				char values[ARRAY_SIZE(driver_name) - 1];
			} driver_name;
			uint16_t _table_bone_mask[10];
			struct
			{ // table BoneMask
				int32_t _table;
				bool body_part, bone_length, rotation, orientation, head_position, tail_position,
				    linear_velocity, angular_velocity;
			} bone_mask;
		} body;
	} handshake_request_packet =
	    {
	        .body =
	            {
	                ._root = htole32(offsetof(struct handshake_request_packet, bundle)),
	                ._table_bundle =
	                    {
	                        htole16(sizeof(handshake_request_packet.body._table_bundle)),
	                        htole16(sizeof(handshake_request_packet.body.bundle)),
	                        0,
	                        0,
	                        htole16(offsetof(struct handshake_request_packet, bundle.driver_msgs) -
	                                offsetof(struct handshake_request_packet, bundle)),
	                    },
	                .bundle =
	                    {
	                        ._table = htole32(offsetof(struct handshake_request_packet, bundle) -
	                                          offsetof(struct handshake_request_packet, _table_bundle)),
	                        .driver_msgs = htole32(offsetof(struct handshake_request_packet, driver_msgs) -
	                                               offsetof(struct handshake_request_packet, bundle.driver_msgs)),
	                    },
	                .driver_msgs =
	                    {
	                        .length = htole32(ARRAY_SIZE(handshake_request_packet.body.driver_msgs.values)),
	                        .values = {htole32(offsetof(struct handshake_request_packet, driver_msgs_0) -
	                                           offsetof(struct handshake_request_packet, driver_msgs.values[0]))},
	                    },
	                ._table_driver_msgs_0 =
	                    {
	                        htole16(sizeof(handshake_request_packet.body._table_driver_msgs_0)),
	                        htole16(sizeof(handshake_request_packet.body.driver_msgs_0) -
	                                sizeof(handshake_request_packet.body.driver_msgs_0._pad)),
	                        0,
	                        0,
	                        htole16(offsetof(struct handshake_request_packet, driver_msgs_0.message_type) -
	                                offsetof(struct handshake_request_packet, driver_msgs_0)),
	                        htole16(offsetof(struct handshake_request_packet, driver_msgs_0.message) -
	                                offsetof(struct handshake_request_packet, driver_msgs_0)),
	                    },
	                .driver_msgs_0 =
	                    {
	                        ._table = htole32(offsetof(struct handshake_request_packet, driver_msgs_0) -
	                                          offsetof(struct handshake_request_packet, _table_driver_msgs_0)),
	                        .message = htole32(offsetof(struct handshake_request_packet, request) -
	                                           offsetof(struct handshake_request_packet, driver_msgs_0.message)),
	                        .message_type = SOLARXR_DRIVER_MESSAGE_TYPE_HANDSHAKE_REQUEST,
	                    },
	                ._table_request =
	                    {
	                        htole16(sizeof(handshake_request_packet.body._table_request)),
	                        htole16(sizeof(handshake_request_packet.body.request)),
	                        htole16(offsetof(struct handshake_request_packet, request.driver_name) -
	                                offsetof(struct handshake_request_packet, request)),
	                        htole16(offsetof(struct handshake_request_packet, request.bone_mask) -
	                                offsetof(struct handshake_request_packet, request)),
	                    },
	                .request =
	                    {
	                        ._table = htole32(offsetof(struct handshake_request_packet, request) -
	                                          offsetof(struct handshake_request_packet, _table_request)),
	                        .driver_name = htole32(offsetof(struct handshake_request_packet, driver_name) -
	                                               offsetof(struct handshake_request_packet, request.driver_name)),
	                        .bone_mask = htole32(offsetof(struct handshake_request_packet, bone_mask) -
	                                             offsetof(struct handshake_request_packet, request.bone_mask)),
	                    },
	                .driver_name =
	                    {
	                        .length = htole32(ARRAY_SIZE(handshake_request_packet.body.driver_name.values)),
	                    },
	                ._table_bone_mask =
	                    {
	                        htole16(sizeof(handshake_request_packet.body._table_bone_mask)),
	                        htole16(sizeof(handshake_request_packet.body.bone_mask)),
	                        htole16(offsetof(struct handshake_request_packet, bone_mask.body_part) -
	                                offsetof(struct handshake_request_packet, bone_mask)),
	                        htole16(offsetof(struct handshake_request_packet, bone_mask.bone_length) -
	                                offsetof(struct handshake_request_packet, bone_mask)),
	                        htole16(offsetof(struct handshake_request_packet, bone_mask.rotation) -
	                                offsetof(struct handshake_request_packet, bone_mask)),
	                        htole16(offsetof(struct handshake_request_packet, bone_mask.orientation) -
	                                offsetof(struct handshake_request_packet, bone_mask)),
	                        htole16(offsetof(struct handshake_request_packet, bone_mask.head_position) -
	                                offsetof(struct handshake_request_packet, bone_mask)),
	                        htole16(offsetof(struct handshake_request_packet, bone_mask.tail_position) -
	                                offsetof(struct handshake_request_packet, bone_mask)),
	                        htole16(offsetof(struct handshake_request_packet, bone_mask.linear_velocity) -
	                                offsetof(struct handshake_request_packet, bone_mask)),
	                        htole16(offsetof(struct handshake_request_packet, bone_mask.angular_velocity) -
	                                offsetof(struct handshake_request_packet, bone_mask)),
	                    },
	                .bone_mask =
	                    {
	                        ._table = htole32(offsetof(struct handshake_request_packet, bone_mask) -
	                                          offsetof(struct handshake_request_packet, _table_bone_mask)),
	                        .body_part = true,
	                        .bone_length = true,
	                        .rotation = true,
	                        .orientation = true,
	                        .head_position = true,
	                        .tail_position = true,
	                        .linear_velocity = true,
	                        .angular_velocity = true,
	                    },
	            },
	    };
	memcpy(handshake_request_packet.body.driver_name.values, driver_name,
	       sizeof(handshake_request_packet.body.driver_name.values));

	struct
	{
		uint8_t head[sizeof(struct solarxr_ipc_message)];
		struct settings_request_packet
		{
			uint32_t _root;
			uint16_t _table_bundle[4];
			struct
			{ // table MessageBundle
				int32_t _table;
				uint32_t rpc_msgs; // vector*
			} bundle;
			struct
			{ // vector<table RpcMessageHeader>
				uint32_t length;
				uint32_t values[1]; // table*
			} rpc_msgs;
			uint16_t _table_rpc_msgs_0[6];
			struct
			{ // table RpcMessageHeader
				int32_t _table;
				uint32_t message;     // table*
				uint8_t message_type; // enum RpcMessage
				uint8_t _pad[3];
			} rpc_msgs_0;
			uint16_t _table_request[2];
			struct
			{ // table BoneRoutingSettingsRequest
				int32_t _table;
			} request;
		} body;
	} const settings_request_packet = {
	    .body =
	        {
	            ._root = htole32(offsetof(struct settings_request_packet, bundle)),
	            ._table_bundle =
	                {
	                    htole16(sizeof(settings_request_packet.body._table_bundle)),
	                    htole16(sizeof(settings_request_packet.body.bundle)),
	                    0,
	                    htole16(offsetof(struct settings_request_packet, bundle.rpc_msgs) -
	                            offsetof(struct settings_request_packet, bundle)),
	                },
	            .bundle =
	                {
	                    ._table = htole32(offsetof(struct settings_request_packet, bundle) -
	                                      offsetof(struct settings_request_packet, _table_bundle)),
	                    .rpc_msgs = htole32(offsetof(struct settings_request_packet, rpc_msgs) -
	                                        offsetof(struct settings_request_packet, bundle.rpc_msgs)),
	                },
	            .rpc_msgs =
	                {
	                    .length = htole32(ARRAY_SIZE(settings_request_packet.body.rpc_msgs.values)),
	                    .values = {htole32(offsetof(struct settings_request_packet, rpc_msgs_0) -
	                                       offsetof(struct settings_request_packet, rpc_msgs.values[0]))},
	                },
	            ._table_rpc_msgs_0 =
	                {
	                    htole16(sizeof(settings_request_packet.body._table_rpc_msgs_0)),
	                    htole16(sizeof(settings_request_packet.body.rpc_msgs_0) -
	                            sizeof(settings_request_packet.body.rpc_msgs_0._pad)),
	                    0,
	                    0,
	                    htole16(offsetof(struct settings_request_packet, rpc_msgs_0.message_type) -
	                            offsetof(struct settings_request_packet, rpc_msgs_0)),
	                    htole16(offsetof(struct settings_request_packet, rpc_msgs_0.message) -
	                            offsetof(struct settings_request_packet, rpc_msgs_0)),
	                },
	            .rpc_msgs_0 =
	                {
	                    ._table = htole32(offsetof(struct settings_request_packet, rpc_msgs_0) -
	                                      offsetof(struct settings_request_packet, _table_rpc_msgs_0)),
	                    .message = htole32(offsetof(struct settings_request_packet, request) -
	                                       offsetof(struct settings_request_packet, rpc_msgs_0.message)),
	                    .message_type = SOLARXR_RPC_MESSAGE_TYPE_BONE_ROUTING_SETTINGS_REQUEST,
	                },
	            ._table_request =
	                {
	                    htole16(sizeof(settings_request_packet.body._table_request)),
	                    htole16(sizeof(settings_request_packet.body.request)),
	                },
	            .request =
	                {
	                    ._table = htole32(offsetof(struct settings_request_packet, request) -
	                                      offsetof(struct settings_request_packet, _table_request)),
	                },
	        },
	};

	bool lost_connection = false;
	while (solarxr_ipc_socket_is_connected(&device->socket)) {
		if (!solarxr_ipc_socket_wait_timeout(&device->socket, -1)) {
			if (!solarxr_ipc_socket_is_connected(&device->socket)) {
				// We were woke up from wait_timeout when destroying the socket
				break;
			}

			if (!lost_connection) {
				U_LOG_IFL_E(debug_get_log_option_solarxr_log(),
				            "solarxr_ipc_socket_wait_timeout failed; attempting socket reconnect");
				lost_connection = true;
				device->shook_hands = false;
				atomic_store(&device->enabled_bones, 0);
				for (size_t i = 0; i < ARRAY_SIZE(device->tracker_refs); ++i) {
					struct solarxr_generic_tracker *const tracker = device->tracker_refs[i];
					if (tracker == NULL)
						break;

					os_mutex_lock(&tracker->mutex);
					tracker->charge = -1.f;
					os_mutex_unlock(&tracker->mutex);
				}
				feeder_mark_unsent(&device->feeder);
			}

			//! @todo Maybe write a better way to silence the logs than passing in RAW log level?
			const struct sockaddr_un path = solarxr_ipc_socket_find(U_LOGGING_RAW, "SlimeVRRpc");
			if (solarxr_ipc_socket_reconnect(&device->socket, path)) {
				lost_connection = false;
			} else {
				os_nanosleep(U_TIME_1S_IN_NS);
			}
			continue;
		}
		const struct span buffer = {solarxr_ipc_socket_receive(&device->socket), device->socket.buffer};

		if (buffer.length == 0) {
			U_LOG_IFL_E(debug_get_log_option_solarxr_log(), "solarxr_ipc_socket_receive failed");
			continue;
		}

		XRT_TRACE_IDENT(solarxr_network_frame);

		struct solarxr_message_bundle bundle;
		if (!read_solarxr_message_bundle(&bundle, buffer.data, buffer.length,
		                                 (const solarxr_message_bundle_t *)buffer.data)) {
			U_LOG_IFL_E(debug_get_log_option_solarxr_log(), "read_solarxr_message_bundle() failed");
			continue;
		}

		struct flatbuffers_vector_solarxr_bone_route routes = {0};
		for (uint32_t i = 0; i < bundle.rpc_msgs.length; ++i) {
			struct solarxr_rpc_message_header header;
			if (!read_solarxr_rpc_message_header(&header, buffer.data, buffer.length,
			                                     &bundle.rpc_msgs.data[i])) {
				U_LOG_IFL_E(debug_get_log_option_solarxr_log(),
				            "read_solarxr_rpc_message_header() failed");
				continue;
			}
			if (header.message_type != SOLARXR_RPC_MESSAGE_TYPE_BONE_ROUTING_SETTINGS_RESPONSE) {
				continue;
			}

			routes.length = header.message.bone_routing_settings_response.routes.length;
			routes.data = header.message.bone_routing_settings_response.routes.data;
		}

		if (routes.length != 0) {
			uint64_t bones = 0;
			for (size_t i = 0; i < routes.length; ++i) {
				struct solarxr_bone_route route;
				if (!read_solarxr_bone_route(&route, buffer.data, buffer.length, &routes.data[i])) {
					U_LOG_IFL_E(debug_get_log_option_solarxr_log(),
					            "read_solarxr_bone_route() failed");
					continue;
				}

				for (size_t j = 0; j < route.outputs.length; ++j) {
					if (route.outputs.data[j] == SOLARXR_ROUTING_OUTPUT_DRIVER) {
						bones |= 1llu << route.bone;
						break;
					}
				}
			}

			const uint64_t old_bones = atomic_exchange(&device->enabled_bones, bones);
			if (old_bones != bones) {
				U_LOG_IFL_D(debug_get_log_option_solarxr_log(), "Bone mask set to 0x%016" PRIx64,
				            bones);
			}
		}

		if (bundle.driver_msgs.length == 0) {
			continue;
		}

		struct flatbuffers_vector_solarxr_bone bones = {0};
		for (uint32_t i = 0; i < bundle.driver_msgs.length; ++i) {
			struct solarxr_driver_message_header header;
			if (!read_solarxr_driver_message_header(&header, buffer.data, buffer.length,
			                                        &bundle.driver_msgs.data[i])) {
				U_LOG_IFL_E(debug_get_log_option_solarxr_log(),
				            "read_solarxr_data_feed_message_header() failed");
				continue;
			}

			switch (header.message_type) {
			case SOLARXR_DRIVER_MESSAGE_TYPE_HANDSHAKE_AVAILABLE:
				U_LOG_IFL_D(debug_get_log_option_solarxr_log(),
				            "Got HandshakeAvailable, sending handshake request");
				solarxr_ipc_socket_send_raw(
				    &device->socket, (const uint8_t *)&handshake_request_packet,
				    solarxr_ipc_message_inline((uint8_t *)&handshake_request_packet,
				                               sizeof(handshake_request_packet)));
				break;
			case SOLARXR_DRIVER_MESSAGE_TYPE_HANDSHAKE_RESPONSE:
				if (header.message.handshake_response.status == SOLARXR_HANDSHAKE_STATUS_ACCEPTED) {
					U_LOG_IFL_I(debug_get_log_option_solarxr_log(), "Shook hands with server");
					device->shook_hands = true;
					solarxr_ipc_socket_send_raw(
					    &device->socket, (const uint8_t *)&settings_request_packet,
					    solarxr_ipc_message_inline((uint8_t *)&settings_request_packet,
					                               sizeof(settings_request_packet)));
				} else {
					U_LOG_IFL_W(
					    debug_get_log_option_solarxr_log(), "Driver handshake rejected (status=%s)",
					    solarxr_handshake_status_to_str(header.message.handshake_response.status));
					device->shook_hands = false;
					atomic_store(&device->enabled_bones, 0);
					for (size_t i = 0; i < ARRAY_SIZE(device->tracker_refs); ++i) {
						struct solarxr_generic_tracker *const tracker = device->tracker_refs[i];
						if (tracker == NULL)
							break;

						os_mutex_lock(&tracker->mutex);
						tracker->charge = -1.f;
						os_mutex_unlock(&tracker->mutex);
					}
					feeder_mark_unsent(&device->feeder);
				}
				break;
			case SOLARXR_DRIVER_MESSAGE_TYPE_ADD_TRACKER_RESPONSE: {
				enum solarxr_add_tracker_status status = header.message.add_tracker_response.status;
				if (status == SOLARXR_ADD_TRACKER_STATUS_CREATED ||
				    status == SOLARXR_ADD_TRACKER_STATUS_ALREADY_EXISTS) {
					feeder_tracker_add_acked(&device->feeder, header.reply_to,
					                         header.message.add_tracker_response.tracker_id);
				} else {
					U_LOG_IFL_W(debug_get_log_option_solarxr_log(),
					            "Server rejected AddTrackerRequest for tracker with id=%" PRIx32
					            " (status=%s)",
					            header.reply_to, solarxr_add_tracker_status_to_str(status));
				}
				break;
			}
			case SOLARXR_DRIVER_MESSAGE_TYPE_SKELETON_UPDATE:
				if (header.message.skeleton_update.bones.length != 0) {
					bones.length = header.message.skeleton_update.bones.length;
					bones.data = header.message.skeleton_update.bones.data;
				}
				break;
			case SOLARXR_DRIVER_MESSAGE_TYPE_BONE_BATTERY_UPDATE:
				for (size_t i = 0; i < ARRAY_SIZE(device->tracker_refs); ++i) {
					struct solarxr_generic_tracker *const tracker = device->tracker_refs[i];
					if (tracker == NULL)
						break;
					if (tracker->role != header.message.bone_battery_update.bone)
						continue;

					os_mutex_lock(&tracker->mutex);
					tracker->charge =
					    (float)header.message.bone_battery_update.battery_level / 100.f;
					tracker->charging = header.message.bone_battery_update.charging;
					os_mutex_unlock(&tracker->mutex);
					break;
				}
				break;
			default:;
			}
		}

		if (device->shook_hands) {
			feeder_send_feedback(&device->feeder);
		}

		if (bones.length != 0) {
			atomic_store(&device->timestamp, device->socket.timestamp);

			struct solarxr_bone newBones[ARRAY_SIZE(device->bones)] = {0};
			for (size_t i = 0; i < bones.length; ++i) {
				struct solarxr_bone bone;
				if (!read_solarxr_bone(&bone, buffer.data, buffer.length, &bones.data[i])) {
					U_LOG_IFL_E(debug_get_log_option_solarxr_log(), "read_solarxr_bone() failed");
					continue;
				}

				if (bone.body_part >= ARRAY_SIZE(device->bones)) {
					static bool _once = false;
					if (!_once) {
						_once = true;
						U_LOG_IFL_W(debug_get_log_option_solarxr_log(),
						            "Unexpected SolarXR BodyPart %u", (unsigned)bone.body_part);
					}
					continue;
				}
				memcpy(&newBones[bone.body_part], &bone, sizeof(bone));
			}
			solarxr_device_handle_bones(device, newBones);
		}
	}

	return NULL;
}

static void
solarxr_device_destroy(struct xrt_device *xdev)
{
	struct solarxr_device *const device = solarxr_device(xdev);
	assert(device != NULL);
	os_mutex_lock(&device->mutex);

	bool tracker_destroying = false;
	for (uint32_t i = 0; i < ARRAY_SIZE(device->tracker_refs); ++i) {
		struct solarxr_generic_tracker *const tracker = device->tracker_refs[i];
		if (tracker == NULL) {
			continue;
		}

		os_mutex_lock(&tracker->mutex);
		if (tracker->parent != NULL) {
			device->tracker_refs[i] = NULL;
		} else {
			tracker_destroying = true;
		}

		tracker->parent = NULL;
		tracker->history = NULL;
		os_mutex_unlock(&tracker->mutex);
	}

	while (tracker_destroying) {
		// a different thread was blocked in `solarxr_generic_tracker_destroy()`
		os_mutex_unlock(&device->mutex);
		sched_yield();
		os_mutex_lock(&device->mutex);

		tracker_destroying = false;
		for (uint32_t i = 0; i < ARRAY_SIZE(device->tracker_refs); ++i) {
			if (device->tracker_refs[i] != NULL) {
				tracker_destroying = true;
				break;
			}
		}
	}

	solarxr_ipc_socket_destroy(&device->socket);
	os_mutex_unlock(&device->mutex);

	if (!pthread_equal(device->thread.thread, pthread_self())) {
		os_thread_join(&device->thread);
		os_thread_destroy(&device->thread);
	}

	feeder_fini(&device->feeder);

	for (size_t i = 0; i < ARRAY_SIZE(device->trackers); ++i) {
		m_relation_history_destroy(&device->trackers[i]);
	}

	os_mutex_destroy(&device->mutex);
	u_device_free(&device->base);
}

static xrt_result_t
solarxr_generic_tracker_get_tracked_pose(struct xrt_device *const xdev,
                                         const enum xrt_input_name name,
                                         const int64_t at_timestamp_ns,
                                         struct xrt_space_relation *const out_relation)
{
	struct solarxr_generic_tracker *const device = solarxr_generic_tracker(xdev);
	assert(device != NULL);

	os_mutex_lock(&device->mutex);

	*out_relation = (struct xrt_space_relation){0};
	xrt_result_t result = XRT_ERROR_INPUT_UNSUPPORTED; // TODO: adding/removing devices at runtime
	struct solarxr_device *const parent = device->parent;

	if (parent != NULL && (atomic_load(&parent->enabled_bones) & (1llu << device->role)) != 0) {
		assert(device->history != NULL);
		m_relation_history_get(device->history, at_timestamp_ns, out_relation);
		result = XRT_SUCCESS;
	}

	os_mutex_unlock(&device->mutex);
	return result;
}

static xrt_result_t
solarxr_generic_tracker_get_battery_status(struct xrt_device *const xdev,
                                           bool *const out_present,
                                           bool *const out_charging,
                                           float *const out_charge)
{
	struct solarxr_generic_tracker *const device = solarxr_generic_tracker(xdev);
	assert(device != NULL);

	os_mutex_lock(&device->mutex);

	if (device->charge == -1.f) {
		*out_present = false;
		goto end;
	}

	*out_present = true;
	*out_charge = device->charge;
	*out_charging = device->charging;

end:
	os_mutex_unlock(&device->mutex);
	return XRT_SUCCESS;
}

static void
solarxr_generic_tracker_destroy(struct xrt_device *const xdev)
{
	struct solarxr_generic_tracker *const device = solarxr_generic_tracker(xdev);
	assert(device != NULL);

	os_mutex_lock(&device->mutex);

	struct solarxr_device *const parent = device->parent;
	device->parent = NULL;
	device->history = NULL;

	os_mutex_unlock(&device->mutex);

	if (parent != NULL) {
		os_mutex_lock(&parent->mutex);
		parent->tracker_refs[device->index] = NULL;
		os_mutex_unlock(&parent->mutex);
	}

	os_mutex_destroy(&device->mutex);
	u_device_free(&device->base);
}

uint32_t
solarxr_device_create_xdevs(struct xrt_tracking_origin *const tracking_origin,
                            struct xrt_device *out_xdevs[const],
                            uint32_t out_xdevs_cap)
{
	if (out_xdevs_cap == 0) {
		return 0;
	}

	struct solarxr_device *device;
	if (out_xdevs_cap > 1 + ARRAY_SIZE(device->trackers)) {
		out_xdevs_cap = 1 + ARRAY_SIZE(device->trackers);
	}

	device = U_DEVICE_ALLOCATE(struct solarxr_device, U_DEVICE_ALLOC_NO_FLAGS, 2, 0);
	device->base.name = XRT_DEVICE_FB_BODY_TRACKING;
	device->base.device_type = XRT_DEVICE_TYPE_BODY_TRACKER;
	strncpy(device->base.str, "SolarXR IPC Connection", sizeof(device->base.str) - 1);
	device->base.tracking_origin = tracking_origin;

	if (device->base.tracking_origin == NULL) {
		device->base.tracking_origin = &device->standalone_origin;
		device->standalone_origin = (struct xrt_tracking_origin){
		    .name = "SolarXR Bridge",
		    .type = XRT_TRACKING_TYPE_OTHER,
		    .initial_offset = XRT_POSE_IDENTITY,
		};
	}

	device->base.supported.body_tracking = true;
	device->base.update_inputs = solarxr_device_update_inputs;
	device->base.get_body_skeleton = solarxr_device_get_body_skeleton;
	device->base.get_body_joints = solarxr_device_get_body_joints;
	device->base.destroy = solarxr_device_destroy;
	device->base.inputs[0].name = XRT_INPUT_FB_BODY_TRACKING;
	device->base.inputs[1].name = XRT_INPUT_META_FULL_BODY_TRACKING;
	device->thread.thread = pthread_self();

	const bool use_trackers = (out_xdevs_cap >= 2);
	device->use_trackers = use_trackers;

	solarxr_ipc_socket_init(&device->socket, debug_get_log_option_solarxr_log());

	uint32_t trackers_len = 0;
	struct solarxr_generic_tracker *trackers[ARRAY_SIZE(device->trackers)];

	// `solarxr_device_destroy()` asserts unless both have attempted initialization
	if (((int)!feeder_init(&device->feeder, &device->socket) | os_mutex_init(&device->mutex)) != 0) {
		goto fail;
	}

	const struct sockaddr_un path = solarxr_ipc_socket_find(debug_get_log_option_solarxr_log(), "SlimeVRRpc");
	if (!solarxr_ipc_socket_connect(&device->socket, path)) {
		goto fail;
	}

	memcpy(device->base.serial, path.sun_path,
	       MIN(sizeof(device->base.serial) - sizeof(char), sizeof(path.sun_path)));

	if (use_trackers) {
		static const enum solarxr_body_part whitelisted_body_parts[] = {
		    SOLARXR_BODY_PART_UPPER_CHEST,    SOLARXR_BODY_PART_HIP,
		    SOLARXR_BODY_PART_LEFT_UPPER_LEG, SOLARXR_BODY_PART_RIGHT_UPPER_LEG,
		    SOLARXR_BODY_PART_LEFT_FOOT,      SOLARXR_BODY_PART_RIGHT_FOOT,
		    SOLARXR_BODY_PART_LEFT_UPPER_ARM, SOLARXR_BODY_PART_RIGHT_UPPER_ARM,
		    SOLARXR_BODY_PART_LEFT_LOWER_ARM, SOLARXR_BODY_PART_RIGHT_LOWER_ARM,
		};
		uint32_t num_trackers = ARRAY_SIZE(whitelisted_body_parts);

		if (num_trackers > out_xdevs_cap - 1) {
			U_LOG_IFL_E(debug_get_log_option_solarxr_log(),
			            "Not enough xdev slots! Omitting %" PRIu32 " trackers",
			            num_trackers - (out_xdevs_cap - 1));
			num_trackers = out_xdevs_cap - 1;
		}

		for (uint32_t i = 0; i < num_trackers; ++i) {
			const enum solarxr_body_part body_part = whitelisted_body_parts[i];
			const char *const body_part_name = solarxr_body_part_to_str(body_part);
			struct solarxr_generic_tracker *const tracker =
			    U_DEVICE_ALLOCATE(struct solarxr_generic_tracker, U_DEVICE_ALLOC_NO_FLAGS, 1, 0);
			if (os_mutex_init(&tracker->mutex) != 0) {
				u_device_free(&tracker->base);
				goto fail;
			}

			tracker->base.name = XRT_DEVICE_VIVE_TRACKER;
			tracker->base.device_type = XRT_DEVICE_TYPE_GENERIC_TRACKER;
			snprintf(tracker->base.str, sizeof(tracker->base.str), "SolarXR \"%s\" Tracker",
			         body_part_name);
			snprintf(tracker->base.serial, sizeof(tracker->base.serial), "SOLARXR-%s", body_part_name);
			tracker->base.tracking_origin = device->base.tracking_origin;
			tracker->base.supported.orientation_tracking = true;
			tracker->base.supported.position_tracking = true;
			tracker->base.supported.battery_status = true;
			tracker->base.update_inputs = solarxr_generic_tracker_update_inputs;
			tracker->base.get_tracked_pose = solarxr_generic_tracker_get_tracked_pose;
			tracker->base.get_battery_status = solarxr_generic_tracker_get_battery_status;
			tracker->base.destroy = solarxr_generic_tracker_destroy;
			tracker->base.inputs[0].name = XRT_INPUT_GENERIC_TRACKER_POSE;
			tracker->role = body_part;
			tracker->charge = -1.f;

			assert(trackers_len < ARRAY_SIZE(trackers));
			m_relation_history_create(&device->trackers[trackers_len]);
			tracker->parent = device;
			tracker->index = trackers_len;
			tracker->history = device->trackers[trackers_len];
			device->tracker_refs[trackers_len] = tracker;

			trackers[trackers_len++] = tracker;
		}
	}

	if (os_thread_start(&device->thread, solarxr_network_thread, device) != 0) {
		U_LOG_IFL_E(debug_get_log_option_solarxr_log(), "pthread_create() failed");
		goto fail;
	}

	assert(1 + trackers_len <= out_xdevs_cap);
	out_xdevs[0] = &device->base;
	for (uint32_t i = 0; i < trackers_len; ++i) {
		out_xdevs[1 + i] = &trackers[i]->base;
	}
	return 1 + trackers_len;

fail:
	for (uint32_t i = 0; i < trackers_len; ++i) {
		solarxr_generic_tracker_destroy(&trackers[i]->base);
	}

	solarxr_device_destroy(&device->base);
	return 0;
}

bool
solarxr_device_add_feeder_device(struct xrt_device *const device, struct xrt_device *const xdev)
{
	struct solarxr_device *const solarxr = solarxr_device(device);
	if (solarxr == NULL || solarxr_device(xdev) != NULL || solarxr_generic_tracker(xdev) != NULL) {
		return false;
	}
	return feeder_add_device(&solarxr->feeder, xdev);
}

void
solarxr_device_remove_feeder_device(struct xrt_device *const device, struct xrt_device *const xdev)
{
	struct solarxr_device *const solarxr = solarxr_device(device);
	if (solarxr != NULL) {
		feeder_remove_device(&solarxr->feeder, xdev);
	}
}

void
solarxr_device_clear_feeder_devices(struct xrt_device *const device)
{
	struct solarxr_device *const solarxr = solarxr_device(device);
	if (solarxr != NULL) {
		feeder_clear_devices(&solarxr->feeder);
	}
}
