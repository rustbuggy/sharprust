#include "mcdriver.h"

#include "math.h"

#define STOP 90
#define NORMAL_FORWARD 105
#define NORMAL_BACKWARD 65

#ifdef DEBUG
#include "stdio.h"
#endif

MCDriver::MCDriver() {
	state = STATE_IDLE;

	min_front = 0;

	maybe_stuck = false;

	driveCmd.steeringPwm = STOP;
	driveCmd.drivingPwm = STOP;
}

void MCDriver::setSteeringAndSpeed(bc_telemetry_packet_t& telemetry) {
	// Steering
	if (driveCmd.steeringPwm > 130) {
		driveCmd.steeringPwm = 130;
	}
	else if (driveCmd.steeringPwm < 50) {
		driveCmd.steeringPwm = 50;
	}

	// Speed
	if (driveCmd.drivingPwm > NORMAL_FORWARD) {
		driveCmd.drivingPwm = NORMAL_FORWARD;
	}
	else if (driveCmd.drivingPwm < NORMAL_BACKWARD) {
		driveCmd.drivingPwm = NORMAL_BACKWARD;
	}

	// Fill missing telemetry values
	telemetry.steering_pwm = driveCmd.steeringPwm;
	telemetry.driving_pwm = driveCmd.drivingPwm;
}

void MCDriver::calc_mc(bc_telemetry_packet_t& telemetry) {
	// Mass center calculation
	fixed_t a1 = FIXED_Mul(VAL_SQRT_1_DIV_2, telemetry.ir_front_right);
	fixed_t a2 = FIXED_Mul(VAL_SQRT_1_DIV_2, telemetry.ir_front_left);
	fixed_t a3 = telemetry.ir_right;
	fixed_t a4 = telemetry.ir_left;
	fixed_t a5 = telemetry.ir_front;

	min_front = a1;
	if (min_front > a2) {
		min_front = a2;
	}
	if (min_front > a5) {
		min_front = a5;
	}

	// Fill missing telemetry values
	telemetry.mc_x = FIXED_Mul(VAL_1_DIV_5, a1 + a3 - a2 - a4); // TODO: calculate offset from sensor positioning as well
	telemetry.mc_y = FIXED_Mul(VAL_1_DIV_5, a1 + a2 + a5); // TODO: calculate offset from sensor positioning as well
	telemetry.mc_dist = FIXED_FROM_DOUBLE(
		sqrt(FIXED_TO_DOUBLE(FIXED_Mul(telemetry.mc_x, telemetry.mc_x) + FIXED_Mul(telemetry.mc_y, telemetry.mc_y)))); // TODO: get rid of double and sqrt
	telemetry.mc_angle = FIXED_Mul(VAL_RAD_TO_DEG,
		FIXED_FROM_DOUBLE(atan2(FIXED_TO_DOUBLE(telemetry.mc_y), FIXED_TO_DOUBLE(telemetry.mc_x)))); // TODO: get rid of double and ata
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
	calc_mc(telemetry);
	maybe_stuck = (FIXED_TO_INT(telemetry.mc_dist) < 10) || (FIXED_TO_INT(min_front) < 10); // replace it with some kind of sensible state machine to get out of stuck state

	switch (state) {
		case STATE_NORMAL:
			driveCmd.steeringPwm = 90 - (FIXED_TO_INT(telemetry.mc_angle) - 90);
			driveCmd.drivingPwm = NORMAL_FORWARD;

			if (maybe_stuck) {
				if (!stuck_timer.running()) {
					stuck_timer.start(telemetry.time, 3000);
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
			driveCmd.steeringPwm = 90 + (FIXED_TO_INT(telemetry.mc_angle) - 90);
			driveCmd.drivingPwm = NORMAL_BACKWARD;

			if (!stuck_timer.running()) {
				stuck_timer.start(telemetry.time, 3000);
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

	setSteeringAndSpeed(telemetry);

	return driveCmd;
}

