/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file simulator.cpp
 * A device simulator
 */

#include <px4_log.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <px4_posix.h>
#include <pthread.h>
#include <poll.h>
#include <systemlib/err.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <netinet/in.h>
#include "../mavlink/v1.0/common/mavlink.h"

#include "comm_forward.h"
#define SEND_PORT 	14660


// __EXPORT int comm_forward_main(int argc, char *argv[]);

extern "C" { __EXPORT int comm_forward_main(int argc, char *argv[]); }

using namespace std;

namespace px4
{

Comm_forward::Comm_forward()
	// _send_addr(0),
	// _fd(0)
{

}

Comm_forward::~Comm_forward()
{
	// _task_should_exit = true;


}




int Comm_forward::task_spawn(int argc, char *argv[])
{
	

	_task_id = px4_task_spawn_cmd("comm_forward",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_MAX - 5,
				      4000,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Comm_forward *Comm_forward::instantiate(int argc, char *argv[])
{
	
	Comm_forward *instance = nullptr;

	instance = new Comm_forward();

	return instance;
}

int Comm_forward::custom_command(int argc, char *argv[])
{

	return print_usage("unknown command");
}



int Comm_forward::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}
	return 0;
}

void Comm_forward::run()
{
	PX4_INFO("Hello command forward running!");


	/* subscribe to sensor_combined topic */
    int vehicle_pos_fd = orb_subscribe(ORB_ID(vehicle_gps_position));
    /* limit the update rate to 2 Hz */
    orb_set_interval(vehicle_pos_fd, 500);


    struct vehicle_gps_position_s	_vehicle_gps;
    mavlink_hil_gps_t gps;
    
    /* copy all topics first time */
    orb_copy(ORB_ID(vehicle_gps_position), vehicle_pos_fd, &_vehicle_gps);

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[1] = {};
    fds[0].fd = vehicle_pos_fd;
    fds[0].events = POLLIN;

    memset((char *) &_send_addr, 0, sizeof(_send_addr));
	_send_addr.sin_family = AF_INET;
	_send_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	_send_addr.sin_port = htons(SEND_PORT);

	if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		PX4_WARN("create socket failed\n");
		return;
	}
	

    while (!should_exit()) {
    	/* wait for up to 1000ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
		// Let the loop run anyway, don't do `continue` here. 

		} else if (pret < 0) {
			// this is undesirable but not much we can do - might want to flag unhappy status 
			PX4_ERR("poll error %d, %d", pret, errno);
			usleep(10000);
			continue;

		} else {

			orb_copy(ORB_ID(vehicle_gps_position), vehicle_pos_fd, &_vehicle_gps);

			gps.time_usec = _vehicle_gps.time_utc_usec;

			gps.lat = _vehicle_gps.lat;
			gps.lon = _vehicle_gps.lon;
			gps.alt = _vehicle_gps.alt;
			gps.eph = (float)_vehicle_gps.eph / 1e-2f; // from m to cm
			gps.epv = (float)_vehicle_gps.epv / 1e-2f; // from m to cm


			gps.vel = (float)_vehicle_gps.vel_m_s / 1e-2f; // from m/s to cm/s
			gps.vn = _vehicle_gps.vel_n_m_s / 1e-2f; // from m to cm
			gps.ve = _vehicle_gps.vel_e_m_s / 1e-2f; // from m to cm
			gps.vd = _vehicle_gps.vel_d_m_s / 1e-2f; // from m to cm
			gps.cog = (float)(_vehicle_gps.cog_rad) / 3.1415f * (100.0f * 180.0f);

			gps.fix_type = _vehicle_gps.fix_type;
			gps.satellites_visible = _vehicle_gps.satellites_used;

			PX4_INFO("lat is: %f", (double) gps.lat);

			mavlink_message_t msg;
			mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &gps);
			send_mavlink_hil_gps(&msg);

		}
	}
}


void Comm_forward::send_mavlink_hil_gps(const mavlink_message_t *msg) {
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	int packetlen = mavlink_msg_to_send_buffer(buffer, msg);

//	PX4_INFO("SENDING GPS MESSAGES");

	ssize_t len = sendto(_fd, buffer, packetlen, 0, (struct sockaddr *) &_send_addr, sizeof(_send_addr));

	if (len <= 0) {
		PX4_INFO("Failed sending mavlink message\n");
	}

}



} //namespace px4

using namespace px4;



int comm_forward_main(int argc, char *argv[])
{
	//check for logfile env variable
	PX4_INFO("Hello Sky!");
    PX4_INFO("Hello command forward!");

	return Comm_forward::main(argc, argv);
}



// int comm_forward_main(int argc, char *argv[])
// {


// 	struct vehicle_gps_position_s	_gps_pos;

//     PX4_INFO("Hello Sky!");
//     PX4_INFO("Hello command forward!");

//     /* subscribe to sensor_combined topic */
//     int vehicle_pos_fd = orb_subscribe(ORB_ID(vehicle_gps_position));
//     /* limit the update rate to 2 Hz */
//     orb_set_interval(vehicle_pos_fd, 500);


//  //    // /* advertise attitude topic */
//  //    // struct vehicle_attitude_s att;
//  //    // memset(&att, 0, sizeof(att));
//  //    // orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);


//     /* copy all topics first time */
//     orb_copy(ORB_ID(vehicle_gps_position), vehicle_pos_fd, &_gps_pos);

//     /* one could wait for multiple topics with this technique, just using one here */
//     px4_pollfd_struct_t fds[1] = {};
//     fds[0].fd = vehicle_pos_fd;
//     fds[0].events = POLLIN;

//     while (1) {
//     	/* wait for up to 1000ms for data */
// 		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

// 		if (pret == 0) {
// 			 Let the loop run anyway, don't do `continue` here. 

// 		} else if (pret < 0) {
// 			// this is undesirable but not much we can do - might want to flag unhappy status 
// 			PX4_ERR("poll error %d, %d", pret, errno);
// 			usleep(10000);
// 			continue;

// 		} else {

// 			orb_copy(ORB_ID(vehicle_gps_position), vehicle_pos_fd, &_gps_pos);

// 			_gps_pos.timestamp = hrt_absolute_time();
// 			// _gps_pos.lat = gps.lat;
// 			// _gps_pos.lon = gps.lon;
// 			// _gps_pos.alt = gps.alt;
// 			// _gps_pos.eph = (float)gps.eph * 1e-2f;
// 			// _gps_pos.epv = (float)gps.epv * 1e-2f;
// 			// _gps_pos.vel_m_s = (float)(gps.vel) / 100.0f;
// 			// _gps_pos.vel_n_m_s = (float)(gps.vn) / 100.0f;
// 			// _gps_pos.vel_e_m_s = (float)(gps.ve) / 100.0f;
// 			// _gps_pos.vel_d_m_s = (float)(gps.vd) / 100.0f;
// 			// _gps_pos.cog_rad = (float)(gps.cog) * 3.1415f / (100.0f * 180.0f);
// 			// _gps_pos.fix_type = gps.fix_type;
// 			// _gps_pos.satellites_used = gps.satellites_visible;

// 			// timestamp_last = gps.timestamp;
// 			PX4_INFO("lat is: %f", (double) _gps_pos.lat);

// 		}
// 	}

//     return OK;
// }

