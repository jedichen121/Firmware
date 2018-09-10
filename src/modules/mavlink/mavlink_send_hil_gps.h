

#pragma once

#include <px4_posix.h>

#include <stdbool.h>
#ifdef __PX4_NUTTX
#include <nuttx/fs/fs.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <drivers/device/device.h>
#endif
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <pthread.h>
#include <systemlib/mavlink_log.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/telemetry_status.h>

#include "../mavlink/v1.0/common/mavlink.h"
#include <netinet/in.h>

__EXPORT extern void send_mavlink_hil_gps(mavlink_hil_gps_t *gps, int *_fd, struct sockaddr_in *_send_addr);

void send_mavlink_hil_gps(mavlink_hil_gps_t *gps, int *_fd, struct sockaddr_in *_send_addr) {

	mavlink_message_t hil_msg;
	mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &hil_msg, gps);
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	int packetlen = mavlink_msg_to_send_buffer(buffer, &hil_msg);

	ssize_t len = sendto(*_fd, buffer, packetlen, 0, (struct sockaddr *) _send_addr, sizeof(*_send_addr));
	if (len <= 0) {
		PX4_INFO("Failed sending mavlink message\n");
	}
}



