#ifndef __MCDRIVER__H
#define __MCDRIVER__H

#include "driver.h"

typedef enum mc_driver_states_t {
	STATE_IDLE = 1, STATE_NORMAL, STATE_START_BACKING_1, STATE_START_BACKING_2, STATE_BACKING, STATE_BRAKING
} mc_driver_states_t;

// Drive towards mass center of estimated IR reflection points
class MCDriver: public Driver {
protected:
	mc_driver_states_t state;
	Timeouter stuck_timer, last_speed_add_timer;
	bool maybe_stuck;
	fixed_t min_front;
	uint8_t normal_pwm;

	point_t l, fl, f, fr, r;
	int32_t steering;
	fixed_t last_speed_add;

	void _calc_direction(bc_telemetry_packet_t& telemetry);
	void _clamp_steering_and_speed(bc_telemetry_packet_t& telemetry);

public:
	MCDriver();
	drive_cmd_t& drive(bc_telemetry_packet_t& telemetry);
	void set_drive_pwm(uint8_t pwm);
};

#endif // __MCDRIVER__H

