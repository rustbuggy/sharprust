#include "mcdriver.h"

#include "math.h"

#define STOP 90
#define NORMAL_FORWARD 105
#define MAX_FORWARD 120
#define NORMAL_BACKWARD 65

#ifdef DEBUG
#include "stdio.h"
#endif

MCDriver::MCDriver() {
	state = STATE_IDLE;

	min_front = 0;

	maybe_stuck = false;
	normal_pwm = NORMAL_FORWARD;

	driveCmd.steering_pwm = STOP;
	driveCmd.driving_pwm = STOP;
}

void MCDriver::set_drive_pwm(uint8_t pwm) {
	normal_pwm = pwm;
}

void MCDriver::_clamp_steering_and_speed(bc_telemetry_packet_t& telemetry) {
	// Steering
	if (driveCmd.steering_pwm > 135) {
		driveCmd.steering_pwm = 135;
	}
	else if (driveCmd.steering_pwm < 45) {
		driveCmd.steering_pwm = 45;
	}

	// Speed
	if (driveCmd.driving_pwm > MAX_FORWARD) {
		driveCmd.driving_pwm = MAX_FORWARD;
	}
	else if (driveCmd.driving_pwm < NORMAL_BACKWARD) {
		driveCmd.driving_pwm = NORMAL_BACKWARD;
	}

	// Fill missing telemetry values
	telemetry.steering_pwm = driveCmd.steering_pwm;
	telemetry.driving_pwm = driveCmd.driving_pwm;
}

void MCDriver::_calc_direction(bc_telemetry_packet_t& telemetry) {
	fixed_t a1 = FIXED_Mul(VAL_SQRT_1_DIV_2, telemetry.ir_front_left);
	fixed_t a2 = FIXED_Mul(VAL_SQRT_1_DIV_2, telemetry.ir_front_right);

	l.x = -telemetry.ir_left; // l.y always 0
	fl.x = -a1 - 163840; // -2.5cm
	fl.y = a1 + 294912; // +4.5cm
	f.y = telemetry.ir_front + 327680; // +5cm, f.x always 0
	fr.x = a2 + 163840; // +2.5cm
	fr.y = a2 + 294912; // +4.5cm
	r.x = telemetry.ir_right; // r.y always 0

	min_front = fl.y;
	if (min_front > f.y) {
		min_front = f.y;
	}
	if (min_front > fr.y) {
		min_front = fr.y;
	}

	// Fill missing telemetry values
	fixed_t inv_wsum = FIXED_Div(FIXED_ONE,
		telemetry.ir_left + telemetry.ir_front_left + telemetry.ir_front + telemetry.ir_front_right + telemetry.ir_right);
	telemetry.mc.x = FIXED_Mul(
		FIXED_Mul(telemetry.ir_left, l.x) + FIXED_Mul(telemetry.ir_front_left, fl.x) + FIXED_Mul(telemetry.ir_front_right, fr.x)
				+ FIXED_Mul(telemetry.ir_right, r.x), inv_wsum);
	telemetry.mc.y = FIXED_Mul(
		FIXED_Mul(telemetry.ir_front_left, fl.y) + FIXED_Mul(telemetry.ir_front, f.y) + FIXED_Mul(telemetry.ir_front_right, fr.y),
		inv_wsum);

	telemetry.mc_dist = FIXED_FROM_DOUBLE(
		sqrt(FIXED_TO_DOUBLE(FIXED_Mul(telemetry.mc.x, telemetry.mc.x) + FIXED_Mul(telemetry.mc.y, telemetry.mc.y)))); // TODO: get rid of double and sqrt
	telemetry.mc_angle = FIXED_Mul(VAL_RAD_TO_DEG,
		FIXED_FROM_DOUBLE(atan2(FIXED_TO_DOUBLE(telemetry.mc.y), FIXED_TO_DOUBLE(telemetry.mc.x)))); // TODO: get rid of double and atan2

#ifdef DEBUG
	printf("%2d  %2d  %2d  %2d  %2d  |  x=%4d  y=%4d  d=%9d  a=%9d  |  ",
			a1, a2, a3, a4, a5,
			telemetry.mc_x,
			telemetry.mc_y,
			telemetry.mc_dist,
			telemetry.mc_angle
	);
#endif
}

drive_cmd_t& MCDriver::drive(bc_telemetry_packet_t& telemetry) {
	_calc_direction(telemetry);
	maybe_stuck = (FIXED_TO_INT(telemetry.mc_dist) < 10) || (FIXED_TO_INT(min_front) < 10);

	switch (state) {
		case STATE_NORMAL:
			driveCmd.steering_pwm = 90 - (FIXED_TO_INT(telemetry.mc_angle) - 90);
			driveCmd.driving_pwm = normal_pwm;

			if (maybe_stuck) {
				if (!stuck_timer.running()) {
					stuck_timer.start(telemetry.time, 1000);
				}
				else if (stuck_timer.triggered(telemetry.time)) {
					state = STATE_STUCK;
					stuck_timer.stop();
				}
			}
			else {
				stuck_timer.stop();
			}
			break;

		case STATE_STUCK:
			driveCmd.steering_pwm = 90 + (FIXED_TO_INT(telemetry.mc_angle) - 90);
			driveCmd.driving_pwm = NORMAL_BACKWARD;

			if (!stuck_timer.running()) {
				stuck_timer.start(telemetry.time, 1000);
			}
			else if (!maybe_stuck || stuck_timer.triggered(telemetry.time)) {
				stuck_timer.stop();
				state = STATE_NORMAL;
			}
			break;

		case STATE_IDLE:
		default:
			state = STATE_NORMAL;
			break;
	}

	_clamp_steering_and_speed(telemetry);

	return driveCmd;
}
