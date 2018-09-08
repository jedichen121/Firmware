

#include <px4_log.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/simplex.h>


__EXPORT int simplex_main(int argc, char *argv[]);


int simplex_main(int argc, char *argv[])
{
	struct simplex_s	_simplex;
	memset(&_simplex, 0, sizeof(_simplex));

	orb_advert_t _simplex_pub = orb_advertise(ORB_ID(simplex), &_simplex);

	_simplex.timestamp = hrt_absolute_time();
	_simplex.simplex_switch = false;
	_simplex.safety_start = true;
	orb_publish(ORB_ID(simplex), _simplex_pub, &_simplex);

    PX4_INFO("Hello Sky!");
    return OK;
}

