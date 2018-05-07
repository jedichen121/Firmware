/****************************************************************************
 *
 * Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file df_ms5611_wrapper.cpp
 * Lightweight driver to access the MS5611 of the DriverFramework.
 */

#include <px4_config.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <px4_getopt.h>
#include <errno.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>

#include <board_config.h>

#include <ms5611/MS5611.hpp>
#include <DevMgr.hpp>

#include "../mavlink/v1.0/common/mavlink.h"
#include "../mavlink/v1.0/common/mavlink_msg_hil_sensor_baro.h"
#include <netinet/in.h>


extern "C" { __EXPORT int df_ms5611_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfMS5611Wrapper : public MS5611
{
public:
	DfMS5611Wrapper();
	~DfMS5611Wrapper();


	/**
	 * Start automatic measurement.
	 *
	 * @return 0 on success
	 */
	int		start();

	/**
	 * Stop automatic measurement.
	 *
	 * @return 0 on success
	 */
	int		stop();

private:
	int _publish(struct baro_sensor_data &data);
	void send_mavlink_message2(const mavlink_message_t *message, const int destination_port);
	orb_advert_t		_baro_topic;

	int _baro_orb_class_instance;

	perf_counter_t _baro_sample_perf;

	mavlink_hil_sensor_baro_t hil_sensor_baro_;

	int _fd;
	sockaddr_in _con_send_addr2;

};

DfMS5611Wrapper::DfMS5611Wrapper() :
	MS5611(BARO_DEVICE_PATH),
	_baro_topic(nullptr),
	_baro_orb_class_instance(-1),
	_baro_sample_perf(perf_alloc(PC_ELAPSED, "df_baro_read"))
{
}

DfMS5611Wrapper::~DfMS5611Wrapper()
{
	perf_free(_baro_sample_perf);
}

int DfMS5611Wrapper::start()
{
	/* Init device and start sensor. */
	int ret = init();

	if (ret != 0) {
		PX4_ERR("MS5611 init fail: %d", ret);
		return ret;
	}

	ret = MS5611::start();

	if (ret != 0) {
		PX4_ERR("MS5611 start fail: %d", ret);
		return ret;
	}
	if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		PX4_WARN("create socket failed");
//			return;
	}

	return 0;
}

int DfMS5611Wrapper::stop()
{
	/* Stop sensor. */
	int ret = MS5611::stop();

	if (ret != 0) {
		PX4_ERR("MS5611 stop fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfMS5611Wrapper::_publish(struct baro_sensor_data &data)
{
	perf_begin(_baro_sample_perf);

	baro_report baro_report = {};
	baro_report.timestamp = hrt_absolute_time();

	baro_report.pressure = data.pressure_pa / 100.0f; // convert to mbar
	baro_report.temperature = data.temperature_c;

	// TODO: verify this, it's just copied from the MS5611 driver.

	// Constant for now
	const double MSL_PRESSURE_KPA = 101325.0 / 1000.0;

	/* tropospheric properties (0-11km) for standard atmosphere */
	const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const double g  = 9.80665;	/* gravity constant in m/s/s */
	const double R  = 287.05;	/* ideal gas constant in J/kg/K */

	/* current pressure at MSL in kPa */
	double p1 = MSL_PRESSURE_KPA;

	/* measured pressure in kPa */
	double p = static_cast<double>(data.pressure_pa) / 1000.0;

	/*
	 * Solve:
	 *
	 *     /        -(aR / g)     \
	 *    | (p / p1)          . T1 | - T1
	 *     \                      /
	 * h = -------------------------------  + h1
	 *                   a
	 */
	baro_report.altitude = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;

	// TODO: when is this ever blocked?
	if (!(m_pub_blocked)) {

		if (_baro_topic == nullptr) {
			_baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &baro_report,
							  &_baro_orb_class_instance, ORB_PRIO_DEFAULT);

		} else {
			orb_publish(ORB_ID(sensor_baro), _baro_topic, &baro_report);
		}

		//@zivy
		hil_sensor_baro_.time_usec = baro_report.timestamp;
		hil_sensor_baro_.abs_pressure = baro_report.pressure;
		hil_sensor_baro_.diff_pressure = baro_report.pressure;
		hil_sensor_baro_.pressure_alt = baro_report.altitude;
		hil_sensor_baro_.temperature = baro_report.temperature;
		mavlink_message_t msg;
		mavlink_msg_hil_sensor_baro_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_sensor_baro_);
		send_mavlink_message2(&msg,14660);

	}

	perf_end(_baro_sample_perf);

	return 0;
}

void DfMS5611Wrapper::send_mavlink_message2(const mavlink_message_t *message, const int destination_port) {

	// try to setup udp socket for communcation with simulator

	memset((char *) &_con_send_addr2, 0, sizeof(_con_send_addr2));
	_con_send_addr2.sin_family = AF_INET;
	_con_send_addr2.sin_addr.s_addr = htonl(INADDR_ANY);


	if (destination_port != 0) {
		_con_send_addr2.sin_port = htons(destination_port);
	}

//	if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
//		PX4_WARN("create socket failed");
//		return;
//	}


	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	int packetlen = mavlink_msg_to_send_buffer(buffer, message);

//	PX4_INFO("SENDING GPS MESSAGES");

	ssize_t len = sendto(_fd, buffer, packetlen, 0, (struct sockaddr *) &_con_send_addr2, sizeof(_con_send_addr2));
	if (len <= 0) {
		PX4_INFO("Failed sending mavlink message\n");
	}

//	for (int i = 0; i < len; i++) {
//		printf("%i ", buffer[i]);
//	}printf("\n");
}

;


namespace df_ms5611_wrapper
{

DfMS5611Wrapper *g_dev = nullptr;

int start(/* enum Rotation rotation */);
int stop();
int info();
void usage();

int start(/*enum Rotation rotation*/)
{
	g_dev = new DfMS5611Wrapper(/*rotation*/);

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfMS5611Wrapper object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("DfMS5611Wrapper start failed");
		return ret;
	}

	// Open the IMU sensor
	DevHandle h;
	DevMgr::getHandle(BARO_DEVICE_PATH, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s (%d)",
			    BARO_DEVICE_PATH, h.getError());
		return -1;
	}

	DevMgr::releaseHandle(h);

	return 0;
}

int stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	int ret = g_dev->stop();

	if (ret != 0) {
		PX4_ERR("driver could not be stopped");
		return ret;
	}

	delete g_dev;
	g_dev = nullptr;
	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_DEBUG("state @ %p", g_dev);

	return 0;
}

void
usage()
{
	PX4_WARN("Usage: df_ms5611_wrapper 'start', 'info', 'stop'");
}

} // namespace df_ms5611_wrapper


int
df_ms5611_wrapper_main(int argc, char *argv[])
{
	int ret = 0;
	int myoptind = 1;

	if (argc <= 1) {
		df_ms5611_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = df_ms5611_wrapper::start();
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_ms5611_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_ms5611_wrapper::info();
	}

	else {
		df_ms5611_wrapper::usage();
		return 1;
	}

	return ret;
}
