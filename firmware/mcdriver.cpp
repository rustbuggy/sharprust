#include "mcdriver.h"

#define STEERING_MAX 150
#define STEERING_MIN 30
#define STEERING_NEUTRAL 90
#define STOP 95
#define NORMAL_FORWARD 105
#define MAX_FORWARD 120
#define NORMAL_BACKWARD 65
#define MIN_BACKWARD 0

static const fixed VAL_SQRT_1_DIV_2(0.70710678118654752440084436210485);
static const fixed VAL_SQRT_3_DIV_2(0.86602540378443864676372317075294);
static const fixed VAL_1_DIV_45(0.02222222222222222222222222222222);

static const fixed VAL_0_5(0.5);
static const fixed VAL_1(1);
static const fixed VAL_2(2);
static const fixed VAL_3_5(3.5);
static const fixed VAL_5_5(5.5);

#ifdef DEBUG
#include "stdio.h"
#endif

MCDriver::MCDriver() {
	state = STATE_IDLE;

	maybe_stuck = false;
	normal_pwm = MAX_FORWARD;

	drive_cmd.steering_pwm = STEERING_NEUTRAL;
	drive_cmd.driving_pwm = STOP;

	steering = 0;
	last_speed_add_timer.start(1, 1);
}

void MCDriver::set_drive_pwm(uint8_t pwm) {
	normal_pwm = pwm;
}

void MCDriver::_clamp_steering_and_speed(bc_telemetry_packet_t& telemetry) {
	// Steering
	if (drive_cmd.steering_pwm > STEERING_MAX) {
		drive_cmd.steering_pwm = STEERING_MAX;
	}
	else if (drive_cmd.steering_pwm < STEERING_MIN) {
		drive_cmd.steering_pwm = STEERING_MIN;
	}

	// Speed
	if (drive_cmd.driving_pwm > MAX_FORWARD) {
		drive_cmd.driving_pwm = MAX_FORWARD;
	}
	else if (drive_cmd.driving_pwm < MIN_BACKWARD) {
		drive_cmd.driving_pwm = MIN_BACKWARD;
	}

	// Fill missing telemetry values
	telemetry.steering_pwm = drive_cmd.steering_pwm;
	telemetry.driving_pwm = drive_cmd.driving_pwm;
}

void MCDriver::_calc_direction(bc_telemetry_packet_t& telemetry) {
	// left and right 45 degrees from center (y-axis)
	// front_left and front_right 30 degrees from center (y-axis)
	fixed a1 = telemetry.ir_left * VAL_SQRT_1_DIV_2;
	l.x = -(a1 + VAL_3_5);
	l.y = a1 - VAL_1;
	fl.x = -(telemetry.ir_front_left * VAL_0_5 + VAL_2);
	fl.y = telemetry.ir_front_left * VAL_SQRT_3_DIV_2 + VAL_5_5;
	f.y = telemetry.ir_front + VAL_5_5; // f.x always 0
	fr.x = telemetry.ir_front_right * VAL_0_5 + VAL_2;
	fr.y = telemetry.ir_front_right * VAL_SQRT_3_DIV_2 + VAL_5_5;
	fixed a2 = telemetry.ir_right * VAL_SQRT_1_DIV_2;
	r.x = a2 + VAL_3_5;
	r.y = a2 - VAL_1;

	min_front = fl.y;
	if (min_front > f.y) {
		min_front = f.y;
	}
	if (min_front > fr.y) {
		min_front = fr.y;
	}

	// Fill missing telemetry values
	fixed inv_wsum = 1
			/ (telemetry.ir_left + telemetry.ir_front_left + telemetry.ir_front + telemetry.ir_front_right + telemetry.ir_right);
	telemetry.mc.x = (telemetry.ir_left * l.x + telemetry.ir_front_left * fl.x + telemetry.ir_front_right * fr.x + telemetry.ir_right * r.x)
			* inv_wsum;
	telemetry.mc.y = (telemetry.ir_front_left * fl.y + telemetry.ir_front * f.y + telemetry.ir_front_right * fr.y) * inv_wsum;

	telemetry.mc_dist = telemetry.mc.get_distance();
	telemetry.mc_angle = telemetry.mc.get_deg_angle();

#ifdef DEBUG
	printf("%2d  %2d  %2d  %2d  %2d  |  x=%4d  y=%4d  d=%9d  a=%9d  |  ",
			a1, a2, a3, a4, a5,
			telemetry.mc.x,
			telemetry.mc.y,
			telemetry.mc.dist,
			telemetry.mc.angle
	);
#endif
}

drive_cmd_t& MCDriver::drive(bc_telemetry_packet_t& telemetry) {
	fixed turn, speed_add, front_fact, angle_fact;

	_calc_direction(telemetry);
	maybe_stuck = (telemetry.mc_dist < 10) || (min_front < 20);
	turn = telemetry.mc_angle - 90;
	steering = 2 * int(turn);

	switch (state) {
		case STATE_NORMAL:
			// steering calculations
			drive_cmd.steering_pwm = 90 - steering;

			// speed calculations
			speed_add = fixed(normal_pwm - NORMAL_FORWARD);

			// normal operation
			front_fact = (telemetry.ir_front - 20) * 0.01; // correct speed by front distance
			angle_fact = (45 - turn.abs()) * VAL_1_DIV_45; // correct speed by turn angle
			if (front_fact < 0) {
				front_fact = 0;
			}
			else if (front_fact > 1) {
				front_fact = 1;
			}
			if (angle_fact < 0) {
				angle_fact = 0;
			}
			else if (angle_fact > 1) {
				angle_fact = 1;
			}
			speed_add = angle_fact * (front_fact * speed_add);
			drive_cmd.driving_pwm = NORMAL_FORWARD + int(speed_add);

			// for abrupt lowering of speed go to braking state
			if (speed_add < last_speed_add - 5) {
				stuck_timer.stop();
				last_speed_add = speed_add;
				drive_cmd.driving_pwm = MIN_BACKWARD;
				state = STATE_BRAKING;
				break;
			}
			last_speed_add = speed_add;

			// stuck countdown
			if (maybe_stuck) {
				if (!stuck_timer.running()) {
					stuck_timer.start(telemetry.time, 1000);
				}
				else if (stuck_timer.triggered(telemetry.time)) {
					stuck_timer.stop();
					state = STATE_BACKING;
				}
			}
			else {
				stuck_timer.stop();
			}
			break;

		case STATE_BACKING:
			drive_cmd.steering_pwm = 90 + steering;
			drive_cmd.driving_pwm = NORMAL_BACKWARD;

			if (!stuck_timer.running()) {
				stuck_timer.start(telemetry.time, 2000);
			}
			if (!maybe_stuck || stuck_timer.triggered(telemetry.time)) {
				stuck_timer.stop();
				state = STATE_NORMAL;
			}
			break;

		case STATE_BRAKING:
			drive_cmd.steering_pwm = 90 - steering;
			drive_cmd.driving_pwm = NORMAL_BACKWARD;

			if (!stuck_timer.running()) {
				stuck_timer.start(telemetry.time, 100);
			}
			else if (stuck_timer.triggered(telemetry.time)) {
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

	return drive_cmd;
}
