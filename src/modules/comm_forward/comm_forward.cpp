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


// __EXPORT int comm_forward_main(int argc, char *argv[]);

extern "C" { __EXPORT int comm_forward_main(int argc, char *argv[]); }


// class Comm_forward
// {
// 	Comm_forward();

// 	virtual ~Comm_forward();


// private:





// }



int comm_forward_main(int argc, char *argv[])
{


	struct vehicle_gps_position_s	_gps_pos;

    PX4_INFO("Hello Sky!");
    PX4_INFO("Hello command forward!");

    /* subscribe to sensor_combined topic */
    int vehicle_pos_fd = orb_subscribe(ORB_ID(vehicle_gps_position));
    /* limit the update rate to 2 Hz */
    orb_set_interval(vehicle_pos_fd, 500);


 //    // /* advertise attitude topic */
 //    // struct vehicle_attitude_s att;
 //    // memset(&att, 0, sizeof(att));
 //    // orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);


    /* copy all topics first time */
    orb_copy(ORB_ID(vehicle_gps_position), vehicle_pos_fd, &_gps_pos);

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[1] = {};
    fds[0].fd = vehicle_pos_fd;
    fds[0].events = POLLIN;

    while (1) {
    	/* wait for up to 1000ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			/* Let the loop run anyway, don't do `continue` here. */

		} else if (pret < 0) {
			// this is undesirable but not much we can do - might want to flag unhappy status 
			PX4_ERR("poll error %d, %d", pret, errno);
			usleep(10000);
			continue;

		} else {

			orb_copy(ORB_ID(vehicle_gps_position), vehicle_pos_fd, &_gps_pos);

			_gps_pos.timestamp = hrt_absolute_time();
			// _gps_pos.lat = gps.lat;
			// _gps_pos.lon = gps.lon;
			// _gps_pos.alt = gps.alt;
			// _gps_pos.eph = (float)gps.eph * 1e-2f;
			// _gps_pos.epv = (float)gps.epv * 1e-2f;
			// _gps_pos.vel_m_s = (float)(gps.vel) / 100.0f;
			// _gps_pos.vel_n_m_s = (float)(gps.vn) / 100.0f;
			// _gps_pos.vel_e_m_s = (float)(gps.ve) / 100.0f;
			// _gps_pos.vel_d_m_s = (float)(gps.vd) / 100.0f;
			// _gps_pos.cog_rad = (float)(gps.cog) * 3.1415f / (100.0f * 180.0f);
			// _gps_pos.fix_type = gps.fix_type;
			// _gps_pos.satellites_used = gps.satellites_visible;

			// timestamp_last = gps.timestamp;
			PX4_INFO("lat is: %f", (double) _gps_pos.lat);

		}
	}

    return OK;
}