#ifndef __MCDRIVER__H
#define __MCDRIVER__H

#include "driver.h"

typedef enum mc_driver_states_t {
	STATE_IDLE = 1, STATE_NORMAL, STATE_STUCK
} mc_driver_states_t;

class Timeouter {
private:
	uint32_t trig_time;

public:
	Timeouter() :
			trig_time(0) {
	}

	void start(uint32_t time, uint32_t timeout) {
		trig_time = time + timeout;
	}

	void stop() {
		trig_time = 0;
	}

	bool running() {
		return trig_time > 0;
	}

	bool triggered(uint32_t time) {
		return time > trig_time;
	}
};

// Drive towards mass center of estimated IR reflection points
class MCDriver: public Driver {
protected:
	mc_driver_states_t state;
	Timeouter stuck_timer;
	bool maybe_stuck;
	fixed_t min_front;
	uint8_t normal_pwm;

	void _calc_mc(bc_telemetry_packet_t& telemetry);
	void _clamp_steering_and_speed(bc_telemetry_packet_t& telemetry);

public:
	MCDriver();
	drive_cmd_t& drive(bc_telemetry_packet_t& telemetry);
	void set_drive_pwm(uint8_t pwm);
};

#endif // __MCDRIVER__H

