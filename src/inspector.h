#pragma once

#include <dronecode_sdk/dronecode_sdk.h>

#define MAVLINK_USE_MESSAGE_INFO
#include <mavlink/v2.0/common/mavlink.h>

#include <map>
#include <mutex>

/**
 * @class Inspector
 */
class Inspector {
public:
	Inspector(dronecode_sdk::DronecodeSDK& sdk);
	~Inspector();

	void run();
private:

	struct MessageInfo {
		int count{1};
		mavlink_message_t last_message;
	};

	bool mavlink_message_receive_callback(const void* message);

	void printMessage(const mavlink_message_t& mav_message) const;
	void printField(const mavlink_message_t& mav_message, const mavlink_field_info_t& field, int offset=0) const;

	std::map<uint32_t, MessageInfo> _messages;
	std::mutex _messages_mutex;
};
