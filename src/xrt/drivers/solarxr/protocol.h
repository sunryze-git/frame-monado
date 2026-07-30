// Copyright 2025, rcelyte
// Copyright 2026, Sapphire
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Parser for a small subset of the SolarXR Flatbuffers protocol, as defined at
 * https://github.com/SlimeVR/SolarXR-Protocol
 * @ingroup drv_solarxr
 */

#pragma once
#include "xrt/xrt_defines.h"
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t flatbuffers_uoffset_t; // little-endian byte order

enum solarxr_body_part
{
	SOLARXR_BODY_PART_NONE = 0,
	SOLARXR_BODY_PART_HEAD = 1,
	SOLARXR_BODY_PART_NECK = 2,
	SOLARXR_BODY_PART_UPPER_CHEST = 3,
	SOLARXR_BODY_PART_LOWER_CHEST = 4,
	SOLARXR_BODY_PART_UPPER_WAIST = 5,
	SOLARXR_BODY_PART_LOWER_WAIST = 6,
	SOLARXR_BODY_PART_HIP = 7,
	SOLARXR_BODY_PART_LEFT_UPPER_LEG = 8,
	SOLARXR_BODY_PART_RIGHT_UPPER_LEG = 9,
	SOLARXR_BODY_PART_LEFT_LOWER_LEG = 10,
	SOLARXR_BODY_PART_RIGHT_LOWER_LEG = 11,
	SOLARXR_BODY_PART_LEFT_FOOT = 12,
	SOLARXR_BODY_PART_RIGHT_FOOT = 13,
	SOLARXR_BODY_PART_LEFT_UPPER_ARM = 14,
	SOLARXR_BODY_PART_RIGHT_UPPER_ARM = 15,
	SOLARXR_BODY_PART_LEFT_LOWER_ARM = 16,
	SOLARXR_BODY_PART_RIGHT_LOWER_ARM = 17,
	SOLARXR_BODY_PART_LEFT_HAND = 18,
	SOLARXR_BODY_PART_RIGHT_HAND = 19,
	SOLARXR_BODY_PART_LEFT_SHOULDER = 20,
	SOLARXR_BODY_PART_RIGHT_SHOULDER = 21,
	SOLARXR_BODY_PART_LEFT_THUMB_METACARPAL = 22,
	SOLARXR_BODY_PART_LEFT_THUMB_PROXIMAL = 23,
	SOLARXR_BODY_PART_LEFT_THUMB_DISTAL = 24,
	SOLARXR_BODY_PART_LEFT_INDEX_PROXIMAL = 25,
	SOLARXR_BODY_PART_LEFT_INDEX_INTERMEDIATE = 26,
	SOLARXR_BODY_PART_LEFT_INDEX_DISTAL = 27,
	SOLARXR_BODY_PART_LEFT_MIDDLE_PROXIMAL = 28,
	SOLARXR_BODY_PART_LEFT_MIDDLE_INTERMEDIATE = 29,
	SOLARXR_BODY_PART_LEFT_MIDDLE_DISTAL = 30,
	SOLARXR_BODY_PART_LEFT_RING_PROXIMAL = 31,
	SOLARXR_BODY_PART_LEFT_RING_INTERMEDIATE = 32,
	SOLARXR_BODY_PART_LEFT_RING_DISTAL = 33,
	SOLARXR_BODY_PART_LEFT_LITTLE_PROXIMAL = 34,
	SOLARXR_BODY_PART_LEFT_LITTLE_INTERMEDIATE = 35,
	SOLARXR_BODY_PART_LEFT_LITTLE_DISTAL = 36,
	SOLARXR_BODY_PART_RIGHT_THUMB_METACARPAL = 37,
	SOLARXR_BODY_PART_RIGHT_THUMB_PROXIMAL = 38,
	SOLARXR_BODY_PART_RIGHT_THUMB_DISTAL = 39,
	SOLARXR_BODY_PART_RIGHT_INDEX_PROXIMAL = 40,
	SOLARXR_BODY_PART_RIGHT_INDEX_INTERMEDIATE = 41,
	SOLARXR_BODY_PART_RIGHT_INDEX_DISTAL = 42,
	SOLARXR_BODY_PART_RIGHT_MIDDLE_PROXIMAL = 43,
	SOLARXR_BODY_PART_RIGHT_MIDDLE_INTERMEDIATE = 44,
	SOLARXR_BODY_PART_RIGHT_MIDDLE_DISTAL = 45,
	SOLARXR_BODY_PART_RIGHT_RING_PROXIMAL = 46,
	SOLARXR_BODY_PART_RIGHT_RING_INTERMEDIATE = 47,
	SOLARXR_BODY_PART_RIGHT_RING_DISTAL = 48,
	SOLARXR_BODY_PART_RIGHT_LITTLE_PROXIMAL = 49,
	SOLARXR_BODY_PART_RIGHT_LITTLE_INTERMEDIATE = 50,
	SOLARXR_BODY_PART_RIGHT_LITTLE_DISTAL = 51,
	SOLARXR_BODY_PART_LEFT_BIG_TOE = 52,
	SOLARXR_BODY_PART_LEFT_INDEX_TOE = 53,
	SOLARXR_BODY_PART_LEFT_MIDDLE_TOE = 54,
	SOLARXR_BODY_PART_LEFT_RING_TOE = 55,
	SOLARXR_BODY_PART_LEFT_LITTLE_TOE = 56,
	SOLARXR_BODY_PART_RIGHT_BIG_TOE = 57,
	SOLARXR_BODY_PART_RIGHT_INDEX_TOE = 58,
	SOLARXR_BODY_PART_RIGHT_MIDDLE_TOE = 59,
	SOLARXR_BODY_PART_RIGHT_RING_TOE = 60,
	SOLARXR_BODY_PART_RIGHT_LITTLE_TOE = 61,
	SOLARXR_BODY_PART_MAX_ENUM,
};

static inline const char *
solarxr_body_part_to_str(enum solarxr_body_part body_part)
{
	switch (body_part) {
	case SOLARXR_BODY_PART_NONE: return "NONE";
	case SOLARXR_BODY_PART_HEAD: return "HEAD";
	case SOLARXR_BODY_PART_NECK: return "NECK";
	case SOLARXR_BODY_PART_UPPER_CHEST: return "UPPER_CHEST";
	case SOLARXR_BODY_PART_LOWER_CHEST: return "LOWER_CHEST";
	case SOLARXR_BODY_PART_UPPER_WAIST: return "UPPER_WAIST";
	case SOLARXR_BODY_PART_LOWER_WAIST: return "LOWER_WAIST";
	case SOLARXR_BODY_PART_HIP: return "HIP";
	case SOLARXR_BODY_PART_LEFT_UPPER_LEG: return "LEFT_UPPER_LEG";
	case SOLARXR_BODY_PART_RIGHT_UPPER_LEG: return "RIGHT_UPPER_LEG";
	case SOLARXR_BODY_PART_LEFT_LOWER_LEG: return "LEFT_LOWER_LEG";
	case SOLARXR_BODY_PART_RIGHT_LOWER_LEG: return "RIGHT_LOWER_LEG";
	case SOLARXR_BODY_PART_LEFT_FOOT: return "LEFT_FOOT";
	case SOLARXR_BODY_PART_RIGHT_FOOT: return "RIGHT_FOOT";
	case SOLARXR_BODY_PART_LEFT_UPPER_ARM: return "LEFT_UPPER_ARM";
	case SOLARXR_BODY_PART_RIGHT_UPPER_ARM: return "RIGHT_UPPER_ARM";
	case SOLARXR_BODY_PART_LEFT_LOWER_ARM: return "LEFT_LOWER_ARM";
	case SOLARXR_BODY_PART_RIGHT_LOWER_ARM: return "RIGHT_LOWER_ARM";
	case SOLARXR_BODY_PART_LEFT_HAND: return "LEFT_HAND";
	case SOLARXR_BODY_PART_RIGHT_HAND: return "RIGHT_HAND";
	case SOLARXR_BODY_PART_LEFT_SHOULDER: return "LEFT_SHOULDER";
	case SOLARXR_BODY_PART_RIGHT_SHOULDER: return "RIGHT_SHOULDER";
	case SOLARXR_BODY_PART_LEFT_THUMB_METACARPAL: return "LEFT_THUMB_METACARPAL";
	case SOLARXR_BODY_PART_LEFT_THUMB_PROXIMAL: return "LEFT_THUMB_PROXIMAL";
	case SOLARXR_BODY_PART_LEFT_THUMB_DISTAL: return "LEFT_THUMB_DISTAL";
	case SOLARXR_BODY_PART_LEFT_INDEX_PROXIMAL: return "LEFT_INDEX_PROXIMAL";
	case SOLARXR_BODY_PART_LEFT_INDEX_INTERMEDIATE: return "LEFT_INDEX_INTERMEDIATE";
	case SOLARXR_BODY_PART_LEFT_INDEX_DISTAL: return "LEFT_INDEX_DISTAL";
	case SOLARXR_BODY_PART_LEFT_MIDDLE_PROXIMAL: return "LEFT_MIDDLE_PROXIMAL";
	case SOLARXR_BODY_PART_LEFT_MIDDLE_INTERMEDIATE: return "LEFT_MIDDLE_INTERMEDIATE";
	case SOLARXR_BODY_PART_LEFT_MIDDLE_DISTAL: return "LEFT_MIDDLE_DISTAL";
	case SOLARXR_BODY_PART_LEFT_RING_PROXIMAL: return "LEFT_RING_PROXIMAL";
	case SOLARXR_BODY_PART_LEFT_RING_INTERMEDIATE: return "LEFT_RING_INTERMEDIATE";
	case SOLARXR_BODY_PART_LEFT_RING_DISTAL: return "LEFT_RING_DISTAL";
	case SOLARXR_BODY_PART_LEFT_LITTLE_PROXIMAL: return "LEFT_LITTLE_PROXIMAL";
	case SOLARXR_BODY_PART_LEFT_LITTLE_INTERMEDIATE: return "LEFT_LITTLE_INTERMEDIATE";
	case SOLARXR_BODY_PART_LEFT_LITTLE_DISTAL: return "LEFT_LITTLE_DISTAL";
	case SOLARXR_BODY_PART_RIGHT_THUMB_METACARPAL: return "RIGHT_THUMB_METACARPAL";
	case SOLARXR_BODY_PART_RIGHT_THUMB_PROXIMAL: return "RIGHT_THUMB_PROXIMAL";
	case SOLARXR_BODY_PART_RIGHT_THUMB_DISTAL: return "RIGHT_THUMB_DISTAL";
	case SOLARXR_BODY_PART_RIGHT_INDEX_PROXIMAL: return "RIGHT_INDEX_PROXIMAL";
	case SOLARXR_BODY_PART_RIGHT_INDEX_INTERMEDIATE: return "RIGHT_INDEX_INTERMEDIATE";
	case SOLARXR_BODY_PART_RIGHT_INDEX_DISTAL: return "RIGHT_INDEX_DISTAL";
	case SOLARXR_BODY_PART_RIGHT_MIDDLE_PROXIMAL: return "RIGHT_MIDDLE_PROXIMAL";
	case SOLARXR_BODY_PART_RIGHT_MIDDLE_INTERMEDIATE: return "RIGHT_MIDDLE_INTERMEDIATE";
	case SOLARXR_BODY_PART_RIGHT_MIDDLE_DISTAL: return "RIGHT_MIDDLE_DISTAL";
	case SOLARXR_BODY_PART_RIGHT_RING_PROXIMAL: return "RIGHT_RING_PROXIMAL";
	case SOLARXR_BODY_PART_RIGHT_RING_INTERMEDIATE: return "RIGHT_RING_INTERMEDIATE";
	case SOLARXR_BODY_PART_RIGHT_RING_DISTAL: return "RIGHT_RING_DISTAL";
	case SOLARXR_BODY_PART_RIGHT_LITTLE_PROXIMAL: return "RIGHT_LITTLE_PROXIMAL";
	case SOLARXR_BODY_PART_RIGHT_LITTLE_INTERMEDIATE: return "RIGHT_LITTLE_INTERMEDIATE";
	case SOLARXR_BODY_PART_RIGHT_LITTLE_DISTAL: return "RIGHT_LITTLE_DISTAL";
	case SOLARXR_BODY_PART_LEFT_BIG_TOE: return "LEFT_BIG_TOE";
	case SOLARXR_BODY_PART_LEFT_INDEX_TOE: return "LEFT_INDEX_TOE";
	case SOLARXR_BODY_PART_LEFT_MIDDLE_TOE: return "LEFT_MIDDLE_TOE";
	case SOLARXR_BODY_PART_LEFT_RING_TOE: return "LEFT_RING_TOE";
	case SOLARXR_BODY_PART_LEFT_LITTLE_TOE: return "LEFT_LITTLE_TOE";
	case SOLARXR_BODY_PART_RIGHT_BIG_TOE: return "RIGHT_BIG_TOE";
	case SOLARXR_BODY_PART_RIGHT_INDEX_TOE: return "RIGHT_INDEX_TOE";
	case SOLARXR_BODY_PART_RIGHT_MIDDLE_TOE: return "RIGHT_MIDDLE_TOE";
	case SOLARXR_BODY_PART_RIGHT_RING_TOE: return "RIGHT_RING_TOE";
	case SOLARXR_BODY_PART_RIGHT_LITTLE_TOE: return "RIGHT_LITTLE_TOE";
	default:;
	}
	return "UNKNOWN";
}

enum solarxr_routing_output
{
	SOLARXR_ROUTING_OUTPUT_DRIVER = 0,
};

enum solarxr_tracker_status
{
	SOLARXR_TRACKER_STATUS_NONE = 0,
	SOLARXR_TRACKER_STATUS_DISCONNECTED = 1,
	SOLARXR_TRACKER_STATUS_OK = 2,
	SOLARXR_TRACKER_STATUS_BUSY = 3,
	SOLARXR_TRACKER_STATUS_ERROR = 4,
	SOLARXR_TRACKER_STATUS_OCCLUDED = 5,
	SOLARXR_TRACKER_STATUS_TIMED_OUT = 6,
	SOLARXR_TRACKER_STATUS_SLEEPING = 7,
};

#define DEFINE_FLATBUFFERS_VECTOR(type_, name_)                                                                        \
	static inline struct name_                                                                                     \
	{                                                                                                              \
		uint32_t length;                                                                                       \
		const type_ *data;                                                                                     \
	} name_(const struct flatbuffers_vector from)                                                                  \
	{                                                                                                              \
		return (struct name_){from.length, (const type_ *)from.data};                                          \
	}

DEFINE_FLATBUFFERS_VECTOR(void, flatbuffers_vector)

DEFINE_FLATBUFFERS_VECTOR(char, flatbuffers_vector_char)

DEFINE_FLATBUFFERS_VECTOR(uint8_t, flatbuffers_vector_solarxr_body_part)

DEFINE_FLATBUFFERS_VECTOR(uint8_t, flatbuffers_vector_solarxr_routing_output)

struct solarxr_tracker_info
{ // table solarxr_protocol.data_feed.tracker_data.TrackerInfo
	enum solarxr_body_part body_part;
	struct flatbuffers_vector_char display_name;
};

typedef struct
{
	flatbuffers_uoffset_t offset;
} solarxr_tracker_data_t;

struct solarxr_tracker_data
{ // table solarxr_protocol.data_feed.tracker_data.TrackerData
	bool has_info;
	bool has_rotation;
	bool has_position;
	bool has_linear_acceleration;
	bool has_device_id;
	uint16_t device_id;
	uint16_t tracker_id;
	struct solarxr_tracker_info info;
	struct xrt_quat rotation;
	struct xrt_vec3 position;
	struct xrt_vec3 linear_acceleration;
};

typedef struct
{
	flatbuffers_uoffset_t offset;
} solarxr_device_data_t;

DEFINE_FLATBUFFERS_VECTOR(solarxr_tracker_data_t, flatbuffers_vector_solarxr_tracker_data)

struct solarxr_device_data
{ // table solarxr_protocol.data_feed.device_data.DeviceData
	uint16_t id;
	struct flatbuffers_vector_solarxr_tracker_data trackers; // solarxr_protocol.data_feed.tracker.TrackerData[]
};

typedef struct
{
	flatbuffers_uoffset_t offset;
} solarxr_bone_t;

struct solarxr_bone
{ // table solarxr_protocol.datatypes.Bone
	enum solarxr_body_part body_part;
	float bone_length;
	struct xrt_quat rotation;
	struct xrt_quat orientation;
	struct xrt_vec3 head_position;
	struct xrt_vec3 tail_position;
	struct xrt_vec3 linear_velocity;
	struct xrt_vec3 angular_velocity;
};

DEFINE_FLATBUFFERS_VECTOR(solarxr_device_data_t, flatbuffers_vector_solarxr_device_data)
DEFINE_FLATBUFFERS_VECTOR(solarxr_bone_t, flatbuffers_vector_solarxr_bone)

struct solarxr_data_feed_update
{                                                              // table solarxr_protocol.data_feed.DataFeedUpdate
	struct flatbuffers_vector_solarxr_device_data devices; // solarxr_protocol.data_feed.device_data.DeviceData[]
	struct flatbuffers_vector_solarxr_bone bones;          // solarxr_protocol.datatypes.Bone[]
};

enum solarxr_data_feed_message_type
{
	SOLARXR_DATA_FEED_MESSAGE_POLL_DATA_FEED = 1,
	SOLARXR_DATA_FEED_MESSAGE_START_DATA_FEED = 2,
	SOLARXR_DATA_FEED_MESSAGE_DATA_FEED_UPDATE = 3,
};

union solarxr_data_feed_message { // union solarxr_protocol.data_feed.DataFeedMessage
	struct solarxr_data_feed_update data_feed_update;
};

typedef struct
{
	flatbuffers_uoffset_t offset;
} solarxr_data_feed_message_header_t;

struct solarxr_data_feed_message_header
{ // table solarxr_protocol.data_feed.DataFeedMessageHeader
	enum solarxr_data_feed_message_type message_type;
	union solarxr_data_feed_message message;
};

typedef struct
{
	flatbuffers_uoffset_t offset;
} solarxr_bone_route_t;

struct solarxr_bone_route
{ // table solarxr_protocol.rpc.BoneRoute
	enum solarxr_body_part bone;
	struct flatbuffers_vector_solarxr_routing_output outputs;
};

DEFINE_FLATBUFFERS_VECTOR(solarxr_bone_route_t, flatbuffers_vector_solarxr_bone_route)

struct solarxr_bone_routing_settings_response
{ // table solarxr_protocol.rpc.BoneRoutingSettingsResponse
	struct flatbuffers_vector_solarxr_bone_route routes;
};

enum solarxr_rpc_message_type
{
	SOLARXR_RPC_MESSAGE_TYPE_BONE_ROUTING_SETTINGS_REQUEST = 111,
	SOLARXR_RPC_MESSAGE_TYPE_BONE_ROUTING_SETTINGS_RESPONSE = 112,
};

union solarxr_rpc_message { // union solarxr_protocol.rpc.RpcMessage
	struct solarxr_bone_routing_settings_response bone_routing_settings_response;
};

typedef struct
{
	flatbuffers_uoffset_t offset;
} solarxr_rpc_message_header_t;

struct solarxr_rpc_message_header
{ // table solarxr_protocol.rpc.RpcMessageHeader
	enum solarxr_rpc_message_type message_type;
	union solarxr_rpc_message message;
};

// namespace solarxr_protocol.driver_protocol

enum solarxr_handshake_status
{
	SOLARXR_HANDSHAKE_STATUS_ACCEPTED = 0,
	SOLARXR_HANDSHAKE_STATUS_REJECTED_UNNAMED = 1,
	SOLARXR_HANDSHAKE_STATUS_REJECTED_DUPLICATE = 2,
	SOLARXR_HANDSHAKE_STATUS_REJECTED_DISABLED = 3,
};

static inline const char *
solarxr_handshake_status_to_str(enum solarxr_handshake_status status)
{
	switch (status) {
	case SOLARXR_HANDSHAKE_STATUS_ACCEPTED: return "ACCEPTED";
	case SOLARXR_HANDSHAKE_STATUS_REJECTED_UNNAMED: return "REJECTED_UNNAMED";
	case SOLARXR_HANDSHAKE_STATUS_REJECTED_DUPLICATE: return "REJECTED_DUPLICATE";
	case SOLARXR_HANDSHAKE_STATUS_REJECTED_DISABLED: return "REJECTED_DISABLED";
	default:;
	}
	return "UNKNOWN";
}

struct solarxr_handshake_response
{ // table solarxr_protocol.driver_protocol.HandshakeResponse
	enum solarxr_handshake_status status;
};

enum solarxr_add_tracker_status
{
	SOLARXR_ADD_TRACKER_STATUS_CREATED = 0,
	SOLARXR_ADD_TRACKER_STATUS_ALREADY_EXISTS = 1,
	SOLARXR_ADD_TRACKER_STATUS_ERROR = 2,
};

static inline const char *
solarxr_add_tracker_status_to_str(enum solarxr_add_tracker_status status)
{
	switch (status) {
	case SOLARXR_ADD_TRACKER_STATUS_CREATED: return "CREATED";
	case SOLARXR_ADD_TRACKER_STATUS_ALREADY_EXISTS: return "ALREADY_EXISTS";
	case SOLARXR_ADD_TRACKER_STATUS_ERROR: return "ERROR";
	default:;
	}
	return "UNKNOWN";
}

struct solarxr_add_tracker_response
{ // table solarxr_protocol.driver_protocol.AddTrackerResponse
	enum solarxr_add_tracker_status status;
	uint16_t tracker_id;
};

struct solarxr_skeleton_update
{                                                     // table solarxr_protocol.driver_protocol.SkeletonUpdate
	struct flatbuffers_vector_solarxr_bone bones; // solarxr_protocol.datatypes.Bone[]
};

struct solarxr_bone_battery_update
{ // table solarxr_protocol.driver_protocol.BoneBatteryUpdate
	enum solarxr_body_part bone;
	uint8_t battery_level;
	bool charging;
};

enum solarxr_driver_message_type
{
	SOLARXR_DRIVER_MESSAGE_TYPE_HANDSHAKE_AVAILABLE = 1,
	SOLARXR_DRIVER_MESSAGE_TYPE_HANDSHAKE_REQUEST = 2,
	SOLARXR_DRIVER_MESSAGE_TYPE_HANDSHAKE_RESPONSE = 3,

	SOLARXR_DRIVER_MESSAGE_TYPE_ADD_TRACKER_REQUEST = 4,
	SOLARXR_DRIVER_MESSAGE_TYPE_ADD_TRACKER_RESPONSE = 5,

	SOLARXR_DRIVER_MESSAGE_TYPE_UPDATE_TRACKER_STATUS = 6,
	SOLARXR_DRIVER_MESSAGE_TYPE_UPDATE_TRACKER_BATTERY = 7,
	SOLARXR_DRIVER_MESSAGE_TYPE_UPDATE_TRACKER_POSITION = 8,

	SOLARXR_DRIVER_MESSAGE_TYPE_SKELETON_UPDATE = 9,
	SOLARXR_DRIVER_MESSAGE_TYPE_BONE_BATTERY_UPDATE = 10,
};

union solarxr_driver_message { // union solarxr_protocol.driver_protocol.DriverMessage
	struct solarxr_handshake_response handshake_response;
	struct solarxr_add_tracker_response add_tracker_response;
	struct solarxr_skeleton_update skeleton_update;
	struct solarxr_bone_battery_update bone_battery_update;
};

typedef struct
{
	flatbuffers_uoffset_t offset;
} solarxr_driver_message_header_t;

struct solarxr_driver_message_header
{ // table solarxr_protocol.driver_protocol.DriverMessageHeader
	uint32_t reply_to;
	enum solarxr_driver_message_type message_type;
	union solarxr_driver_message message;
};

typedef struct
{
	flatbuffers_uoffset_t offset;
} solarxr_message_bundle_t;

DEFINE_FLATBUFFERS_VECTOR(solarxr_data_feed_message_header_t, flatbuffers_vector_solarxr_data_feed_message_header)
DEFINE_FLATBUFFERS_VECTOR(solarxr_rpc_message_header_t, flatbuffers_vector_solarxr_rpc_message_header)
DEFINE_FLATBUFFERS_VECTOR(solarxr_driver_message_header_t, flatbuffers_vector_solarxr_driver_message_header)

struct solarxr_message_bundle
{ // table solarxr_protocol.MessageBundle
	struct flatbuffers_vector_solarxr_data_feed_message_header
	    data_feed_msgs; // solarxr_protocol.data_feed.DataFeedMessageHeader[]
	struct flatbuffers_vector_solarxr_rpc_message_header rpc_msgs; // solarxr_protocol.rpc.RpcMessageHeader[]
	struct flatbuffers_vector_solarxr_driver_message_header
	    driver_msgs; // solarxr_protocol.driver_protocol.DriverMessageHeader[]
};

#undef DEFINE_FLATBUFFERS_VECTOR

bool
read_solarxr_message_bundle(struct solarxr_message_bundle *out,
                            const uint8_t buffer[],
                            size_t buffer_len,
                            const solarxr_message_bundle_t *ref);
bool
read_solarxr_data_feed_message_header(struct solarxr_data_feed_message_header *out,
                                      const uint8_t buffer[],
                                      size_t buffer_len,
                                      const solarxr_data_feed_message_header_t *ref);
bool
read_solarxr_rpc_message_header(struct solarxr_rpc_message_header *out,
                                const uint8_t buffer[],
                                size_t buffer_len,
                                const solarxr_rpc_message_header_t *ref);
bool
read_solarxr_driver_message_header(struct solarxr_driver_message_header *out,
                                   const uint8_t buffer[],
                                   size_t buffer_len,
                                   const solarxr_driver_message_header_t *ref);
bool
read_solarxr_device_data(struct solarxr_device_data *out,
                         const uint8_t buffer[],
                         size_t buffer_len,
                         const solarxr_device_data_t *ref);
bool
read_solarxr_tracker_data(struct solarxr_tracker_data *out,
                          const uint8_t buffer[],
                          size_t buffer_len,
                          const solarxr_tracker_data_t *ref);
bool
read_solarxr_bone(struct solarxr_bone *out, const uint8_t buffer[], size_t buffer_len, const solarxr_bone_t *ref);

bool
read_solarxr_bone_route(struct solarxr_bone_route *out,
                        const uint8_t buffer[],
                        size_t buffer_len,
                        const solarxr_bone_route_t *ref);

#ifdef __cplusplus
}
#endif
