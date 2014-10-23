#include "mcdriver.h"

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
	normal_pwm = MAX_FORWARD;
	last_steering = 0;
	max_steering = 0;

	drive_cmd.steering_pwm = STOP;
	drive_cmd.driving_pwm = STOP;
}

void MCDriver::set_drive_pwm(uint8_t pwm) {
	normal_pwm = pwm;
}

void MCDriver::_clamp_steering_and_speed(bc_telemetry_packet_t& telemetry) {
	// Steering
	if (drive_cmd.steering_pwm > 150) {
		drive_cmd.steering_pwm = 150;
	}
	else if (drive_cmd.steering_pwm < 30) {
		drive_cmd.steering_pwm = 30;
	}

	// Speed
	if (drive_cmd.driving_pwm > MAX_FORWARD) {
		drive_cmd.driving_pwm = MAX_FORWARD;
	}
	else if (drive_cmd.driving_pwm < NORMAL_BACKWARD) {
		drive_cmd.driving_pwm = NORMAL_BACKWARD;
	}

	// Fill missing telemetry values
	telemetry.steering_pwm = drive_cmd.steering_pwm;
	telemetry.driving_pwm = drive_cmd.driving_pwm;
}

void MCDriver::_calc_direction(bc_telemetry_packet_t& telemetry) {
	/*
	 // left and right 90 degrees from center (y-axis)
	 // front_left and front_right 30 degrees from center (y-axis)
	 fixed_t a1 = FIXED_Mul(VAL_SQRT_1_DIV_2, telemetry.ir_front_left);
	 fixed_t a2 = FIXED_Mul(VAL_SQRT_1_DIV_2, telemetry.ir_front_right);

	 l.x = -telemetry.ir_left; // l.y always 0
	 fl.x = -a1 - 163840; // -2.5cm
	 fl.y = a1 + 294912; // +4.5cm
	 f.y = telemetry.ir_front + 327680; // +5cm, f.x always 0
	 fr.x = a2 + 163840; // +2.5cm
	 fr.y = a2 + 294912; // +4.5cm
	 r.x = telemetry.ir_right; // r.y always 0
	 */

	// left and right 45 degrees from center (y-axis)
	// front_left and front_right 30 degrees from center (y-axis)
	fixed_t a1 = FIXED_Mul(VAL_SQRT_1_DIV_2, telemetry.ir_left);
	fixed_t a2 = FIXED_Mul(VAL_SQRT_1_DIV_2, telemetry.ir_right);

	l.x = -a1 - 229376; // -3.5cm
	l.y = a1 - 65536; // -1.0cm
	fl.x = -FIXED_Mul(32768, telemetry.ir_front_left) - 131072; // -2.0cm
	fl.y = FIXED_Mul(56756, telemetry.ir_front_left) + 360448; // +5.5cm
	f.y = telemetry.ir_front + 360448; // +5.5cm, f.x always 0
	fr.x = FIXED_Mul(32768, telemetry.ir_front_right) + 131072; // +2.0cm
	fr.y = FIXED_Mul(56756, telemetry.ir_front_right) + 360448; // +5.5cm
	r.x = a2 + 229376; // +3.5cm
	r.y = a2 - 65536; // -1.0cm

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

	telemetry.mc_dist = telemetry.mc.get_distance();
	telemetry.mc_angle = telemetry.mc.get_deg_angle();

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
	fixed_t turn, abs_turn, speed_add, front_fact, angle_fact;

	_calc_direction(telemetry);
	maybe_stuck = (telemetry.mc_dist < 655360) || (min_front < 1310720); // dist < 10cm or min_front < 20cm
	turn = telemetry.mc_angle - 5898240; // -90
	steering = 2 * FIXED_TO_INT(turn);

	switch (state) {
		case STATE_NORMAL:
			// steering calculations
			/*
			 if ((last_steering < 0 && steering >= 0) || (last_steering >= 0 && steering < 0)) {
			 steer_correct = -max_steering / 2;
			 steering += steer_correct;
			 max_steering = steering;
			 }
			 else {
			 steer_correct -= steer_correct > 0 ? 1 : -1;
			 steering += steer_correct;
			 if (ABS(steering) > ABS(max_steering)) {
			 max_steering = steering;
			 }
			 }
			 */
			drive_cmd.steering_pwm = 90 - steering;

			// speed calculations
			speed_add = FIXED_FROM_INT(normal_pwm - NORMAL_FORWARD);
			abs_turn = ABS(turn);
			front_fact = FIXED_Mul(2949120 - abs_turn, 1456); // correct speed by turn angle
			angle_fact = FIXED_Mul(telemetry.ir_front - 1310720, 655); // correct speed by front distance
			if (front_fact < 0) {
				front_fact = 0;
			}
			else if (front_fact > FIXED_ONE) {
				front_fact = FIXED_ONE;
			}
			if (angle_fact < 0) {
				angle_fact = 0;
			}
			else if (angle_fact > FIXED_ONE) {
				angle_fact = FIXED_ONE;
			}
			speed_add = FIXED_Mul(angle_fact, FIXED_Mul(front_fact, speed_add));
			drive_cmd.driving_pwm = NORMAL_FORWARD + FIXED_TO_INT(speed_add);

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
			drive_cmd.steering_pwm = 90 + steering;
			drive_cmd.driving_pwm = NORMAL_BACKWARD;

			if (!stuck_timer.running()) {
				stuck_timer.start(telemetry.time, 2000);
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

	last_steering = steering;

	_clamp_steering_and_speed(telemetry);

	return drive_cmd;
}
