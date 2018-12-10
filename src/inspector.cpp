
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <iostream>
#include <thread>
#include <atomic>
#include <inttypes.h>
#include <vector>
#include <algorithm>

#include <dronecode_sdk/system.h>
#include <dronecode_sdk/dronecode_sdk.h>

#include "inspector.h"

using namespace dronecode_sdk;
using namespace std::this_thread;
using namespace std::chrono;
using namespace std;

#define GREEN_CONSOLE_TEXT "\033[32m" // Turn text on console green
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

int main(int argc, char **argv)
{
	DronecodeSDK dc;
	string connection_url;
	ConnectionResult connection_result;

	atomic<bool> discovered_system{false};
	if (argc == 2) {
		connection_url = argv[1];
		// connection_url format:
		// For TCP : tcp://[server_host][:server_port]
		// For UDP : udp://[bind_host][:bind_port]
		// For Serial : serial:///path/to/serial/dev[:baudrate]
		connection_result = dc.add_any_connection(connection_url);
	} else {
		cout << "Must specify a connection" << endl;
		return 1;
	}

	if (connection_result != ConnectionResult::SUCCESS) {
		cout << "Connection failed: " << connection_result_str(connection_result) << endl;
		return 1;
	}

	//System &system = dc.system();

	cout << "Waiting to discover system..." << endl;
	dc.register_on_discover([&discovered_system](uint64_t uuid) {
		cout << "Discovered system with UUID: " << uuid << endl;
		discovered_system = true;
	});

	// We usually receive heartbeats at 1Hz, therefore we should find a system after a few
	// seconds.
	int counter = 0;
	while (counter++ < 30 && !discovered_system)
		sleep_for(milliseconds(100));

	if (!discovered_system) {
		cout << "No system found, exiting." << endl;
		return 1;
	}

	Inspector inspector(dc);
	inspector.run();

    return 0;
}

Inspector::Inspector(dronecode_sdk::DronecodeSDK& sdk)
{
	sdk.register_mavlink_message_receive_callback(
			bind(&Inspector::mavlink_message_receive_callback, this, placeholders::_1));
}

Inspector::~Inspector()
{
}

void Inspector::run()
{
	auto start = std::chrono::steady_clock::now();
	while (true) {
		sleep_for(milliseconds(100));

		auto now = std::chrono::steady_clock::now();
		auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
		if (duration_ms > 1000) {
			printf("\033[2J\n"); // clear screen

			// print messages in sorted order (according to the update rate)
			vector<MessageInfo> messages_sorted;
			{
				lock_guard<mutex> lock(_messages_mutex);
				for (const auto& message : _messages)
					messages_sorted.push_back(message.second);
				_messages.clear();
			}
			sort(messages_sorted.begin(), messages_sorted.end(),
					[](const MessageInfo& a, const MessageInfo& b) { return a.count < b.count; });
			for (const auto& message : messages_sorted) {
				printf("%i ", (int) (message.count * 1000.f / duration_ms + 0.5f));
				printMessage(message.last_message);
			}
			start = now;
		}
	}
}

bool Inspector::mavlink_message_receive_callback(const void* message)
{
	const mavlink_message_t *mav_message = (const mavlink_message_t*)message;
	if (!mav_message) {
		return false;
	}

	// TODO: enqueue message? or mutex
	// this could come from multiple systems...

	// accumulated stats
	lock_guard<mutex> lock(_messages_mutex);
	auto message_iter = _messages.find(mav_message->msgid);
	if (message_iter == _messages.end()) {
		MessageInfo info;
		info.last_message = * mav_message;
		_messages[mav_message->msgid] = info;
	} else {
		++message_iter->second.count;
	}

	//printMessage(*mav_message);

	return true;
}

void Inspector::printMessage(const mavlink_message_t& mav_message) const
{
	const mavlink_message_info_t * msg_info = mavlink_get_message_info(&mav_message);

	// print message name + all fields
	if (msg_info) {
		printf("%s%s%s (sysid=%i, compid=%i, ", GREEN_CONSOLE_TEXT, msg_info->name,
				NORMAL_CONSOLE_TEXT, mav_message.sysid, mav_message.compid);
		for (unsigned i = 0; i < msg_info->num_fields; ++i) {
			const mavlink_field_info_t& field = msg_info->fields[i];
			printf("%s=", field.name);
			if (field.array_length > 0) {
				if (field.type == MAVLINK_TYPE_CHAR) {
					// it's a string
					// TODO: ensure null-terminated
					printf("%s", (char*)(((uint8_t*)mav_message.payload64)+field.structure_offset));
				} else {
					printf("(");
					for (unsigned j = 0; j < field.array_length; ++j) {
						printField(mav_message, field, j);
						if (j < field.array_length - 1)
							printf(", ");
					}
					printf(")");
				}
			} else {
				printField(mav_message, field);
			}

			if (i < msg_info->num_fields - 1)
				printf(", ");
		}
		printf(")\n");
	} else {
		// unknown message
		printf("MSGID=%i (sysid=%i, compid=%i) len=%i\n", mav_message.msgid,
				mav_message.sysid, mav_message.compid, mav_message.len);
	}
}

void Inspector::printField(const mavlink_message_t& mav_message, const mavlink_field_info_t& field, int offset) const
{
	switch(field.type) {
	case MAVLINK_TYPE_CHAR:
		printf("%c", ((char*)(((uint8_t*)mav_message.payload64)+field.structure_offset))[offset]);
		break;
	case MAVLINK_TYPE_UINT8_T:
		printf("%u", ((uint8_t*)(((uint8_t*)mav_message.payload64)+field.structure_offset))[offset]);
		break;
	case MAVLINK_TYPE_INT8_T:
		printf("%i", ((int8_t*)(((uint8_t*)mav_message.payload64)+field.structure_offset))[offset]);
		break;
	case MAVLINK_TYPE_UINT16_T:
		printf("%u", ((uint16_t*)(((uint8_t*)mav_message.payload64)+field.structure_offset))[offset]);
		break;
	case MAVLINK_TYPE_INT16_T:
		printf("%i", ((int16_t*)(((uint8_t*)mav_message.payload64)+field.structure_offset))[offset]);
		break;
	case MAVLINK_TYPE_UINT32_T:
		printf("%u", ((uint32_t*)(((uint8_t*)mav_message.payload64)+field.structure_offset))[offset]);
		break;
	case MAVLINK_TYPE_INT32_T:
		printf("%i", ((int32_t*)(((uint8_t*)mav_message.payload64)+field.structure_offset))[offset]);
		break;
	case MAVLINK_TYPE_UINT64_T:
		printf("%" PRIu64, ((uint64_t*)(((uint8_t*)mav_message.payload64)+field.structure_offset))[offset]);
		break;
	case MAVLINK_TYPE_INT64_T:
		printf("%" PRId64, ((int64_t*)(((uint8_t*)mav_message.payload64)+field.structure_offset))[offset]);
		break;
	case MAVLINK_TYPE_FLOAT:
		printf("%.3f", (double)(((float*)(((uint8_t*)mav_message.payload64)+field.structure_offset))[offset]));
		break;
	case MAVLINK_TYPE_DOUBLE:
		printf("%.3f", ((double*)(((uint8_t*)mav_message.payload64)+field.structure_offset))[offset]);
		break;
	}
}

