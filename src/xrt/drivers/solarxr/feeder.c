// Copyright 2025, rcelyte
// Copyright 2026, Sapphire
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  SolarXR pose feeder
 * @ingroup drv_solarxr
 */

#include "feeder.h"
#include "protocol.h"
#include "solarxr_ipc_message.h"

#include "util/u_hashmap.h"
#include "util/u_trace_marker.h"
#include "xrt/xrt_device.h"

#include <math.h>

static const char manufacturer[] = "Monado";

struct feeder_device
{
	struct feeder_device *next_to_erase;
	struct xrt_device *xdev;
	enum xrt_input_name input_name;
	uint32_t id;
	bool sent_add_request;
	uint16_t remote_tracker_id; // != 0 means acked and we have a valid ID
	bool destroyed;
	enum solarxr_tracker_status last_status;
	bool battery_charging;
	float battery_charge;
};

// We have to use memcpy to not violate strict aliasing rules
static inline uint32_t
float_to_le32(const float *const src)
{
	uint32_t result;
	static_assert(sizeof(*src) == sizeof(result), "");
	memcpy(&result, src, sizeof(result));
	return htole32(result);
}

static void
serialise_add_tracker_request(const enum u_logging_level log_level,
                              struct feeder_device *const device,
                              uint8_t **const head,
                              const uint8_t *const end)
{
	struct xrt_device *const xdev = device->xdev;

	enum solarxr_body_part role = SOLARXR_BODY_PART_NONE;
	switch (xdev->device_type) {
	case XRT_DEVICE_TYPE_HMD: {
		role = SOLARXR_BODY_PART_HEAD;
	} break;
	case XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER: {
		role = SOLARXR_BODY_PART_RIGHT_HAND;
	} break;
	case XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER: {
		role = SOLARXR_BODY_PART_LEFT_HAND;
	} break;
	default:;
	}

	struct
	{
		// solarxr_ipc_message_write_single takes care of wrapping the message, no need for head
		// uint8_t head[sizeof(struct solarxr_ipc_message)];
		struct add_tracker_request_packet
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
				uint32_t tx_id;
				uint32_t message;     // table*
				uint8_t message_type; // enum DriverMessage
				uint8_t _pad[3];
			} driver_msgs_0;
			uint16_t _table_request[6];
			struct
			{ // table AddTrackerRequest
				int32_t _table;
				uint32_t hardware_identifier; // vector*
				uint32_t display_name;        // vector*
				uint32_t manufacturer;        // vector*
				uint8_t body_part;            // enum BodyPart
				uint8_t _pad[3];
			} request;
			struct
			{ // vector<char>
				uint32_t length;
				char values[ARRAY_SIZE(xdev->serial) - 1];
			} hardware_identifier;
			struct
			{ // vector<char>
				uint32_t length;
				char values[ARRAY_SIZE(xdev->str) - 1];
			} display_name;
			struct
			{ // vector<char>
				uint32_t length;
				char values[ARRAY_SIZE(manufacturer) - 1];
			} manufacturer;
		} body;
	} add_tracker_request_packet =
	    {
	        .body =
	            {
	                ._root = htole32(offsetof(struct add_tracker_request_packet, bundle)),
	                ._table_bundle =
	                    {
	                        htole16(sizeof(add_tracker_request_packet.body._table_bundle)),
	                        htole16(sizeof(add_tracker_request_packet.body.bundle)),
	                        0,
	                        0,
	                        htole16(offsetof(struct add_tracker_request_packet, bundle.driver_msgs) -
	                                offsetof(struct add_tracker_request_packet, bundle)),
	                    },
	                .bundle =
	                    {
	                        ._table = htole32(offsetof(struct add_tracker_request_packet, bundle) -
	                                          offsetof(struct add_tracker_request_packet, _table_bundle)),
	                        .driver_msgs = htole32(offsetof(struct add_tracker_request_packet, driver_msgs) -
	                                               offsetof(struct add_tracker_request_packet, bundle.driver_msgs)),
	                    },
	                .driver_msgs =
	                    {
	                        .length = htole32(ARRAY_SIZE(add_tracker_request_packet.body.driver_msgs.values)),
	                        .values = {htole32(offsetof(struct add_tracker_request_packet, driver_msgs_0) -
	                                           offsetof(struct add_tracker_request_packet, driver_msgs.values[0]))},
	                    },
	                ._table_driver_msgs_0 =
	                    {
	                        htole16(sizeof(add_tracker_request_packet.body._table_driver_msgs_0)),
	                        htole16(sizeof(add_tracker_request_packet.body.driver_msgs_0) -
	                                sizeof(add_tracker_request_packet.body.driver_msgs_0._pad)),
	                        htole16(offsetof(struct add_tracker_request_packet, driver_msgs_0.tx_id) -
	                                offsetof(struct add_tracker_request_packet, driver_msgs_0)),
	                        0,
	                        htole16(offsetof(struct add_tracker_request_packet, driver_msgs_0.message_type) -
	                                offsetof(struct add_tracker_request_packet, driver_msgs_0)),
	                        htole16(offsetof(struct add_tracker_request_packet, driver_msgs_0.message) -
	                                offsetof(struct add_tracker_request_packet, driver_msgs_0)),
	                    },
	                .driver_msgs_0 =
	                    {
	                        ._table = htole32(offsetof(struct add_tracker_request_packet, driver_msgs_0) -
	                                          offsetof(struct add_tracker_request_packet, _table_driver_msgs_0)),
	                        .tx_id = htole32(device->id),
	                        .message = htole32(offsetof(struct add_tracker_request_packet, request) -
	                                           offsetof(struct add_tracker_request_packet, driver_msgs_0.message)),
	                        .message_type = SOLARXR_DRIVER_MESSAGE_TYPE_ADD_TRACKER_REQUEST,
	                    },
	                ._table_request =
	                    {
	                        htole16(sizeof(add_tracker_request_packet.body._table_request)),
	                        htole16(sizeof(add_tracker_request_packet.body.request) -
	                                sizeof(add_tracker_request_packet.body.request._pad)),
	                        htole16(offsetof(struct add_tracker_request_packet, request.hardware_identifier) -
	                                offsetof(struct add_tracker_request_packet, request)),
	                        htole16(offsetof(struct add_tracker_request_packet, request.display_name) -
	                                offsetof(struct add_tracker_request_packet, request)),
	                        htole16(offsetof(struct add_tracker_request_packet, request.manufacturer) -
	                                offsetof(struct add_tracker_request_packet, request)),
	                        htole16(offsetof(struct add_tracker_request_packet, request.body_part) -
	                                offsetof(struct add_tracker_request_packet, request)),
	                    },
	                .request =
	                    {
	                        ._table = htole32(offsetof(struct add_tracker_request_packet, request) -
	                                          offsetof(struct add_tracker_request_packet, _table_request)),
	                        .hardware_identifier =
	                            htole32(offsetof(struct add_tracker_request_packet, hardware_identifier) -
	                                    offsetof(struct add_tracker_request_packet, request.hardware_identifier)),
	                        .display_name = htole32(offsetof(struct add_tracker_request_packet, display_name) -
	                                                offsetof(
	                                                    struct add_tracker_request_packet, request.display_name)),
	                        .manufacturer = htole32(offsetof(struct add_tracker_request_packet, manufacturer) -
	                                                offsetof(
	                                                    struct add_tracker_request_packet, request.manufacturer)),
	                        .body_part = role,
	                    },
	                .hardware_identifier =
	                    {
	                        .length = htole32(strlen(xdev->serial)),
	                    },
	                .display_name =
	                    {
	                        .length = htole32(strlen(xdev->str)),
	                    },
	                .manufacturer =
	                    {
	                        .length = htole32(ARRAY_SIZE(manufacturer) - 1),
	                    },
	            },
	    };
	memcpy(add_tracker_request_packet.body.hardware_identifier.values, xdev->serial,
	       add_tracker_request_packet.body.hardware_identifier.length);
	memcpy(add_tracker_request_packet.body.display_name.values, xdev->str,
	       add_tracker_request_packet.body.display_name.length);
	memcpy(add_tracker_request_packet.body.manufacturer.values, manufacturer,
	       add_tracker_request_packet.body.manufacturer.length);

	solarxr_ipc_message_write_single(head, end, (const uint8_t *)&add_tracker_request_packet,
	                                 sizeof(add_tracker_request_packet));

	U_LOG_IFL_D(log_level, "    \"%s\" [id=%" PRIu32 " serial=\"%s\" role=BodyPart::%s]", xdev->str, device->id,
	            xdev->serial, solarxr_body_part_to_str(role));
}

static void
serialise_device_status_update(struct feeder_device *const device,
                               const enum solarxr_tracker_status status,
                               uint8_t **const head,
                               const uint8_t *const end)
{
	struct
	{
		// solarxr_ipc_message_write_single takes care of wrapping the message, no need for head
		// uint8_t head[sizeof(struct solarxr_ipc_message)];
		struct update_tracker_status_packet
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
			uint16_t _table_update[4];
			struct
			{ // table UpdateTrackerStatus
				int32_t _table;
				uint16_t tracker_id;
				uint8_t status;
			} update;
		} body;
	} const update_tracker_status_packet = {
	    .body =
	        {
	            ._root = htole32(offsetof(struct update_tracker_status_packet, bundle)),
	            ._table_bundle =
	                {
	                    htole16(sizeof(update_tracker_status_packet.body._table_bundle)),
	                    htole16(sizeof(update_tracker_status_packet.body.bundle)),
	                    0,
	                    0,
	                    htole16(offsetof(struct update_tracker_status_packet, bundle.driver_msgs) -
	                            offsetof(struct update_tracker_status_packet, bundle)),
	                },
	            .bundle =
	                {
	                    ._table = htole32(offsetof(struct update_tracker_status_packet, bundle) -
	                                      offsetof(struct update_tracker_status_packet, _table_bundle)),
	                    .driver_msgs = htole32(offsetof(struct update_tracker_status_packet, driver_msgs) -
	                                           offsetof(struct update_tracker_status_packet, bundle.driver_msgs)),
	                },
	            .driver_msgs =
	                {
	                    .length = htole32(ARRAY_SIZE(update_tracker_status_packet.body.driver_msgs.values)),
	                    .values = {htole32(offsetof(struct update_tracker_status_packet, driver_msgs_0) -
	                                       offsetof(struct update_tracker_status_packet, driver_msgs.values[0]))},
	                },
	            ._table_driver_msgs_0 =
	                {
	                    htole16(sizeof(update_tracker_status_packet.body._table_driver_msgs_0)),
	                    htole16(sizeof(update_tracker_status_packet.body.driver_msgs_0) -
	                            sizeof(update_tracker_status_packet.body.driver_msgs_0._pad)),
	                    0,
	                    0,
	                    htole16(offsetof(struct update_tracker_status_packet, driver_msgs_0.message_type) -
	                            offsetof(struct update_tracker_status_packet, driver_msgs_0)),
	                    htole16(offsetof(struct update_tracker_status_packet, driver_msgs_0.message) -
	                            offsetof(struct update_tracker_status_packet, driver_msgs_0)),
	                },
	            .driver_msgs_0 =
	                {
	                    ._table = htole32(offsetof(struct update_tracker_status_packet, driver_msgs_0) -
	                                      offsetof(struct update_tracker_status_packet, _table_driver_msgs_0)),
	                    .message = htole32(offsetof(struct update_tracker_status_packet, update) -
	                                       offsetof(struct update_tracker_status_packet, driver_msgs_0.message)),
	                    .message_type = SOLARXR_DRIVER_MESSAGE_TYPE_UPDATE_TRACKER_STATUS,
	                },
	            ._table_update =
	                {
	                    htole16(sizeof(update_tracker_status_packet.body._table_update)),
	                    htole16(sizeof(update_tracker_status_packet.body.update)),
	                    htole16(offsetof(struct update_tracker_status_packet, update.tracker_id) -
	                            offsetof(struct update_tracker_status_packet, update)),
	                    htole16(offsetof(struct update_tracker_status_packet, update.status) -
	                            offsetof(struct update_tracker_status_packet, update)),
	                },
	            .update =
	                {
	                    ._table = htole32(offsetof(struct update_tracker_status_packet, update) -
	                                      offsetof(struct update_tracker_status_packet, _table_update)),
	                    .tracker_id = htole16(device->remote_tracker_id),
	                    .status = status,
	                },
	        },
	};

	solarxr_ipc_message_write_single(head, end, (const uint8_t *)&update_tracker_status_packet,
	                                 sizeof(update_tracker_status_packet));
}

static void
serialise_device_battery_update(struct feeder_device *const device,
                                const bool charging,
                                const float charge,
                                uint8_t **const head,
                                const uint8_t *const end)
{
	struct
	{
		// solarxr_ipc_message_write_single takes care of wrapping the message, no need for head
		// uint8_t head[sizeof(struct solarxr_ipc_message)];
		struct update_tracker_battery_packet
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
			uint16_t _table_update[5];
			struct
			{ // table UpdateTrackerBattery
				int32_t _table;
				uint16_t tracker_id;
				uint8_t battery_level;
				bool charging;
			} update;
		} body;
	} const update_tracker_battery_packet = {
	    .body =
	        {
	            ._root = htole32(offsetof(struct update_tracker_battery_packet, bundle)),
	            ._table_bundle =
	                {
	                    htole16(sizeof(update_tracker_battery_packet.body._table_bundle)),
	                    htole16(sizeof(update_tracker_battery_packet.body.bundle)),
	                    0,
	                    0,
	                    htole16(offsetof(struct update_tracker_battery_packet, bundle.driver_msgs) -
	                            offsetof(struct update_tracker_battery_packet, bundle)),
	                },
	            .bundle =
	                {
	                    ._table = htole32(offsetof(struct update_tracker_battery_packet, bundle) -
	                                      offsetof(struct update_tracker_battery_packet, _table_bundle)),
	                    .driver_msgs = htole32(offsetof(struct update_tracker_battery_packet, driver_msgs) -
	                                           offsetof(struct update_tracker_battery_packet, bundle.driver_msgs)),
	                },
	            .driver_msgs =
	                {
	                    .length = htole32(ARRAY_SIZE(update_tracker_battery_packet.body.driver_msgs.values)),
	                    .values = {htole32(offsetof(struct update_tracker_battery_packet, driver_msgs_0) -
	                                       offsetof(struct update_tracker_battery_packet, driver_msgs.values[0]))},
	                },
	            ._table_driver_msgs_0 =
	                {
	                    htole16(sizeof(update_tracker_battery_packet.body._table_driver_msgs_0)),
	                    htole16(sizeof(update_tracker_battery_packet.body.driver_msgs_0) -
	                            sizeof(update_tracker_battery_packet.body.driver_msgs_0._pad)),
	                    0,
	                    0,
	                    htole16(offsetof(struct update_tracker_battery_packet, driver_msgs_0.message_type) -
	                            offsetof(struct update_tracker_battery_packet, driver_msgs_0)),
	                    htole16(offsetof(struct update_tracker_battery_packet, driver_msgs_0.message) -
	                            offsetof(struct update_tracker_battery_packet, driver_msgs_0)),
	                },
	            .driver_msgs_0 =
	                {
	                    ._table = htole32(offsetof(struct update_tracker_battery_packet, driver_msgs_0) -
	                                      offsetof(struct update_tracker_battery_packet, _table_driver_msgs_0)),
	                    .message = htole32(offsetof(struct update_tracker_battery_packet, update) -
	                                       offsetof(struct update_tracker_battery_packet, driver_msgs_0.message)),
	                    .message_type = SOLARXR_DRIVER_MESSAGE_TYPE_UPDATE_TRACKER_BATTERY,
	                },
	            ._table_update =
	                {
	                    htole16(sizeof(update_tracker_battery_packet.body._table_update)),
	                    htole16(sizeof(update_tracker_battery_packet.body.update)),
	                    htole16(offsetof(struct update_tracker_battery_packet, update.tracker_id) -
	                            offsetof(struct update_tracker_battery_packet, update)),
	                    htole16(offsetof(struct update_tracker_battery_packet, update.battery_level) -
	                            offsetof(struct update_tracker_battery_packet, update)),
	                    htole16(offsetof(struct update_tracker_battery_packet, update.charging) -
	                            offsetof(struct update_tracker_battery_packet, update)),
	                },
	            .update =
	                {
	                    ._table = htole32(offsetof(struct update_tracker_battery_packet, update) -
	                                      offsetof(struct update_tracker_battery_packet, _table_update)),
	                    .tracker_id = htole16(device->remote_tracker_id),
	                    .battery_level = (uint8_t)(charge * 100.f),
	                    .charging = charging,
	                },
	        },
	};

	solarxr_ipc_message_write_single(head, end, (const uint8_t *)&update_tracker_battery_packet,
	                                 sizeof(update_tracker_battery_packet));
}

static void
serialise_device_pose_update(struct feeder_device *const device,
                             const struct xrt_space_relation *const relation,
                             uint8_t **const head,
                             const uint8_t *const end)
{
	const bool send_rotation = relation->relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
	const bool send_position = relation->relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT;
	const bool send_angular_velocity = relation->relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
	const bool send_linear_velocity = relation->relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;

	struct
	{
		// solarxr_ipc_message_write_single takes care of wrapping the message, no need for head
		// uint8_t head[sizeof(struct solarxr_ipc_message)];
		struct update_tracker_position_packet
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
			uint16_t _table_update[7];
			struct
			{ // table UpdateTrackerPosition
				int32_t _table;
				uint16_t tracker_id;
				struct
				{
					uint32_t x; // float
					uint32_t y; // float
					uint32_t z; // float
					uint32_t w; // float
				} rotation;
				struct
				{
					uint32_t x; // float
					uint32_t y; // float
					uint32_t z; // float
				} position;
				struct
				{
					uint32_t x; // float
					uint32_t y; // float
					uint32_t z; // float
				} angular_velocity;
				struct
				{
					uint32_t x; // float
					uint32_t y; // float
					uint32_t z; // float
				} linear_velocity;
			} update;
		} body;
	} update_tracker_position_packet = {
	    .body =
	        {
	            ._root = htole32(offsetof(struct update_tracker_position_packet, bundle)),
	            ._table_bundle =
	                {
	                    htole16(sizeof(update_tracker_position_packet.body._table_bundle)),
	                    htole16(sizeof(update_tracker_position_packet.body.bundle)),
	                    0,
	                    0,
	                    htole16(offsetof(struct update_tracker_position_packet, bundle.driver_msgs) -
	                            offsetof(struct update_tracker_position_packet, bundle)),
	                },
	            .bundle =
	                {
	                    ._table = htole32(offsetof(struct update_tracker_position_packet, bundle) -
	                                      offsetof(struct update_tracker_position_packet, _table_bundle)),
	                    .driver_msgs = htole32(offsetof(struct update_tracker_position_packet, driver_msgs) -
	                                           offsetof(struct update_tracker_position_packet, bundle.driver_msgs)),
	                },
	            .driver_msgs =
	                {
	                    .length = htole32(ARRAY_SIZE(update_tracker_position_packet.body.driver_msgs.values)),
	                    .values = {htole32(offsetof(struct update_tracker_position_packet, driver_msgs_0) -
	                                       offsetof(struct update_tracker_position_packet, driver_msgs.values[0]))},
	                },
	            ._table_driver_msgs_0 =
	                {
	                    htole16(sizeof(update_tracker_position_packet.body._table_driver_msgs_0)),
	                    htole16(sizeof(update_tracker_position_packet.body.driver_msgs_0) -
	                            sizeof(update_tracker_position_packet.body.driver_msgs_0._pad)),
	                    0,
	                    0,
	                    htole16(offsetof(struct update_tracker_position_packet, driver_msgs_0.message_type) -
	                            offsetof(struct update_tracker_position_packet, driver_msgs_0)),
	                    htole16(offsetof(struct update_tracker_position_packet, driver_msgs_0.message) -
	                            offsetof(struct update_tracker_position_packet, driver_msgs_0)),
	                },
	            .driver_msgs_0 =
	                {
	                    ._table = htole32(offsetof(struct update_tracker_position_packet, driver_msgs_0) -
	                                      offsetof(struct update_tracker_position_packet, _table_driver_msgs_0)),
	                    .message = htole32(offsetof(struct update_tracker_position_packet, update) -
	                                       offsetof(struct update_tracker_position_packet, driver_msgs_0.message)),
	                    .message_type = SOLARXR_DRIVER_MESSAGE_TYPE_UPDATE_TRACKER_POSITION,
	                },
	            ._table_update =
	                {
	                    htole16(sizeof(update_tracker_position_packet.body._table_update)),
	                    htole16(sizeof(update_tracker_position_packet.body.update)),
	                    htole16(offsetof(struct update_tracker_position_packet, update.tracker_id) -
	                            offsetof(struct update_tracker_position_packet, update)),
	                    send_rotation ? htole16(offsetof(struct update_tracker_position_packet, update.rotation) -
	                                            offsetof(struct update_tracker_position_packet, update))
	                                  : 0,
	                    send_position ? htole16(offsetof(struct update_tracker_position_packet, update.position) -
	                                            offsetof(struct update_tracker_position_packet, update))
	                                  : 0,
	                    send_angular_velocity
	                        ? htole16(offsetof(struct update_tracker_position_packet, update.angular_velocity) -
	                                  offsetof(struct update_tracker_position_packet, update))
	                        : 0,
	                    send_linear_velocity
	                        ? htole16(offsetof(struct update_tracker_position_packet, update.linear_velocity) -
	                                  offsetof(struct update_tracker_position_packet, update))
	                        : 0,
	                },
	            .update =
	                {
	                    ._table = htole32(offsetof(struct update_tracker_position_packet, update) -
	                                      offsetof(struct update_tracker_position_packet, _table_update)),
	                    .tracker_id = htole16(device->remote_tracker_id),
	                },
	        },
	};

	if (send_rotation) {
		update_tracker_position_packet.body.update.rotation.x = float_to_le32(&relation->pose.orientation.x);
		update_tracker_position_packet.body.update.rotation.y = float_to_le32(&relation->pose.orientation.y);
		update_tracker_position_packet.body.update.rotation.z = float_to_le32(&relation->pose.orientation.z);
		update_tracker_position_packet.body.update.rotation.w = float_to_le32(&relation->pose.orientation.w);
	}

	if (send_position) {
		update_tracker_position_packet.body.update.position.x = float_to_le32(&relation->pose.position.x);
		update_tracker_position_packet.body.update.position.y = float_to_le32(&relation->pose.position.y);
		update_tracker_position_packet.body.update.position.z = float_to_le32(&relation->pose.position.z);
	}

	if (send_angular_velocity) {
		update_tracker_position_packet.body.update.angular_velocity.x =
		    float_to_le32(&relation->angular_velocity.x);
		update_tracker_position_packet.body.update.angular_velocity.y =
		    float_to_le32(&relation->angular_velocity.y);
		update_tracker_position_packet.body.update.angular_velocity.z =
		    float_to_le32(&relation->angular_velocity.z);
	}

	if (send_linear_velocity) {
		update_tracker_position_packet.body.update.linear_velocity.x =
		    float_to_le32(&relation->linear_velocity.x);
		update_tracker_position_packet.body.update.linear_velocity.y =
		    float_to_le32(&relation->linear_velocity.y);
		update_tracker_position_packet.body.update.linear_velocity.z =
		    float_to_le32(&relation->linear_velocity.z);
	}

	solarxr_ipc_message_write_single(head, end, (const uint8_t *)&update_tracker_position_packet,
	                                 sizeof(update_tracker_position_packet));
}

bool
feeder_add_device(struct feeder *const feeder, struct xrt_device *const xdev)
{
	if (xdev == NULL || !xdev->supported.orientation_tracking) {
		return false;
	}
	os_mutex_lock(&feeder->mutex);

	bool result = false;
	// find a suitable pose input to report
	enum xrt_input_name input_name = XRT_INPUT_GENERIC_TRACKER_POSE;
	for (size_t input = 0, input_count = xdev->input_count; input < input_count; ++input) {
		if (XRT_GET_INPUT_TYPE(xdev->inputs[input].name) == XRT_INPUT_TYPE_POSE) {
			input_name = xdev->inputs[input].name;
			break;
		}
	}

	// initialize tracked state for the device
	struct feeder_device *const device = malloc(sizeof(*device));
	if (device == NULL) {
		U_LOG_IFL_E(feeder->socket->log_level, "malloc() failed");
		goto unlock;
	}
	*device = (struct feeder_device){
	    .xdev = xdev,
	    .input_name = input_name,
	    .id = feeder->next_id++,
	    .last_status = UINT8_MAX,
	};
	u_hashmap_int_insert(feeder->devices, (uint64_t)(uintptr_t)xdev, device);

	result = true;
unlock:
	os_mutex_unlock(&feeder->mutex);
	return result;
}

static void
feeder_device_unlink(struct feeder_device *const device,
                     struct xrt_device *const xdev,
                     const enum u_logging_level log_level)
{
	if (device != NULL) {
		device->destroyed = true;
		U_LOG_IFL_D(log_level, "device \"%.*s\" removed", (unsigned)ARRAY_SIZE(xdev->str), xdev->str);
	}
}

void
feeder_remove_device(struct feeder *const feeder, struct xrt_device *const xdev)
{
	os_mutex_lock(&feeder->mutex);

	struct feeder_device *device = NULL;
	u_hashmap_int_find(feeder->devices, (uint64_t)(uintptr_t)xdev, (void **)&device);
	feeder_device_unlink(device, xdev, feeder->socket->log_level);

	os_mutex_unlock(&feeder->mutex);
}

static void
feeder_device_remove_cb(const uint64_t key, const void *const value, void *const userptr)
{
	struct feeder_device *const device = (struct feeder_device *)value;
	if (!device->destroyed) { // xdev pointer may be dangling if destroyed
		feeder_device_unlink(device, (struct xrt_device *)(uintptr_t)key,
		                     ((struct feeder *)userptr)->socket->log_level);
	}
}

void
feeder_clear_devices(struct feeder *const feeder)
{
	os_mutex_lock(&feeder->mutex);
	u_hashmap_int_for_each(feeder->devices, feeder_device_remove_cb, feeder);
	os_mutex_unlock(&feeder->mutex);
}

static void
feeder_device_mark_unsent_cb(const uint64_t key, const void *const value, void *const userptr)
{
	struct feeder_device *const device = (struct feeder_device *)value;
	device->sent_add_request = false;
	device->remote_tracker_id = 0;
	device->last_status = UINT8_MAX;
}

void
feeder_mark_unsent(struct feeder *const feeder)
{
	os_mutex_lock(&feeder->mutex);
	u_hashmap_int_for_each(feeder->devices, feeder_device_mark_unsent_cb, NULL);
	os_mutex_unlock(&feeder->mutex);
}

struct device_acked_userdata
{
	struct feeder *feeder;
	uint32_t id;
	uint16_t tracker_id;
};

static void
feeder_device_acked_cb(const uint64_t key, const void *const value, void *const userptr)
{
	struct feeder_device *const device = (struct feeder_device *)value;
	struct xrt_device *const xdev = device->destroyed ? NULL : (struct xrt_device *)(uintptr_t)key;
	struct device_acked_userdata *const data = (struct device_acked_userdata *)userptr;

	if (xdev == NULL) {
		return;
	}
	if (device->id == data->id) {
		device->remote_tracker_id = data->tracker_id;
		U_LOG_IFL_D(data->feeder->socket->log_level, "Received ack for device \"%s\", remote ID is %" PRIu16,
		            xdev->str, data->tracker_id);
	}
}

void
feeder_tracker_add_acked(struct feeder *const feeder, const uint32_t id, const uint16_t tracker_id)
{
	os_mutex_lock(&feeder->mutex);

	struct device_acked_userdata data = {
	    .feeder = feeder,
	    .id = id,
	    .tracker_id = tracker_id,
	};
	u_hashmap_int_for_each(feeder->devices, feeder_device_acked_cb, &data);

	os_mutex_unlock(&feeder->mutex);
}

struct send_feedback_userdata
{
	struct feeder_device *to_erase;
	timepoint_ns time;
	enum u_logging_level log_level;
	uint8_t *packet_end, packet[0x10000];
};

static void
feeder_device_send_feedback_cb(const uint64_t key, const void *const value, void *const userptr)
{
	struct feeder_device *const device = (struct feeder_device *)value;
	struct xrt_device *const xdev = device->destroyed ? NULL : (struct xrt_device *)(uintptr_t)key;
	struct send_feedback_userdata *const data = (struct send_feedback_userdata *)userptr;
	enum solarxr_tracker_status status = SOLARXR_TRACKER_STATUS_DISCONNECTED;

	XRT_TRACE_MARKER();

	if (xdev == NULL) {
		device->next_to_erase = data->to_erase;
		data->to_erase = device;
	} else {
		if (!device->sent_add_request) {
			serialise_add_tracker_request(data->log_level, device, &data->packet_end,
			                              &data->packet[ARRAY_SIZE(data->packet)]);
			device->sent_add_request = true;
			return;
		} else if (device->remote_tracker_id == 0) {
			// Didn't get ack yet
			return;
		}

		// report meaningful changes in battery level
		bool present = false, charging = false;
		float charge = 0;
		if (xdev->supported.battery_status &&
		    xrt_device_get_battery_status(xdev, &present, &charging, &charge) == XRT_SUCCESS &&
		    (charging != device->battery_charging || fabsf(charge - device->battery_charge) >= 1e-05)) {
			device->battery_charging = charging;
			device->battery_charge = charge;
			serialise_device_battery_update(device, charging, charge, &data->packet_end,
			                                &data->packet[ARRAY_SIZE(data->packet)]);
		}

		// report pose
		struct xrt_space_relation relation = {0};
		assert(xdev->get_tracked_pose != NULL);
		if (xrt_device_get_tracked_pose(xdev, device->input_name, data->time, &relation) == XRT_SUCCESS &&
		    (relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
			serialise_device_pose_update(device, &relation, &data->packet_end,
			                             &data->packet[ARRAY_SIZE(data->packet)]);
			if ((relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) != 0) {
				status = SOLARXR_TRACKER_STATUS_OK;
			}
		}
	}

	// report tracking status
	if (status != device->last_status) {
		device->last_status = status;
		serialise_device_status_update(device, status, &data->packet_end,
		                               &data->packet[ARRAY_SIZE(data->packet)]);
	}
}

void
feeder_send_feedback(struct feeder *const feeder)
{
	XRT_TRACE_MARKER();

	os_mutex_lock(&feeder->mutex);

	struct send_feedback_userdata data;
	data.to_erase = NULL;
	data.time = os_monotonic_get_ns();
	data.log_level = feeder->socket->log_level;
	data.packet_end = data.packet;

	XRT_TRACE_BEGIN(feeder_device_send_feedback_foreach);
	u_hashmap_int_for_each(feeder->devices, feeder_device_send_feedback_cb, &data);
	XRT_TRACE_END(feeder_device_send_feedback_foreach);

	// flush queued messages to socket
	if (data.packet_end != data.packet) {
		if (solarxr_ipc_socket_is_connected(feeder->socket)) {
			solarxr_ipc_socket_send_raw(feeder->socket, data.packet, data.packet_end - data.packet);
		} else {
			// Called during destroy to clear out the hashmap
			// U_LOG_IFL_E(feeder->socket->log_level, "not connected");
		}
	}

	while (data.to_erase != NULL) {
		struct feeder_device *const entry = data.to_erase;
		data.to_erase = entry->next_to_erase;
		u_hashmap_int_erase(feeder->devices, (uint64_t)(uintptr_t)entry->xdev);
		free(entry);
	}

	os_mutex_unlock(&feeder->mutex);
}

bool
feeder_init(struct feeder *const feeder, struct solarxr_ipc_socket *const socket)
{
	feeder->devices = NULL;
	feeder->socket = socket;
	if (os_mutex_init(&feeder->mutex) != 0) {
		U_LOG_IFL_E(socket->log_level, "os_mutex_init() failed");
		return false;
	}
	u_hashmap_int_create(&feeder->devices);
	return true;
}

static void
feeder_device_free_cb(const uint64_t key, const void *const value, void *const userptr)
{
	(void)key;
	(void)userptr;
	free((void *)value);
}

void
feeder_fini(struct feeder *const feeder)
{
	if (feeder->devices != NULL) {
		feeder_clear_devices(feeder);
		feeder_send_feedback(feeder);
		u_hashmap_int_for_each(feeder->devices, feeder_device_free_cb, NULL);
		u_hashmap_int_destroy(&feeder->devices);
	}
	os_mutex_destroy(&feeder->mutex);
}
