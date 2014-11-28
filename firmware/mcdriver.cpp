#include "mcdriver.h"

#define STEERING_MAX 150
#define STEERING_MIN 30
#define STEERING_NEUTRAL 90
#define STOP 95
#define NORMAL_FORWARD 105
#define MAX_FORWARD 116
#define NORMAL_BACKWARD 60
#define MIN_BACKWARD 0

//static const fixed VAL_SQRT_1_DIV_2(0.70710678118654752440084436210485);
static const fixed VAL_SQRT_3_DIV_2(0.86602540378443864676372317075294);
static const fixed VAL_1_DIV_45(0.02222222222222222222222222222222);

static const fixed VAL_0_0(0);
static const fixed VAL_0_5(0.5);
static const fixed VAL_1_0(1);
static const fixed VAL_2_0(2);
static const fixed VAL_3_0(3);
static const fixed VAL_5_0(5);
static const fixed VAL_6_0(6);

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
	// left and right 60 degrees from center (y-axis)
	// front_left and front_right 30 degrees from center (y-axis)
	// coordinates zero between left and right sensor
	l.x = -(telemetry.ir_left * VAL_SQRT_3_DIV_2 + VAL_3_0);
	l.y = telemetry.ir_left * VAL_0_5;
	fl.x = -(telemetry.ir_front_left * VAL_0_5 + VAL_2_0);
	fl.y = telemetry.ir_front_left * VAL_SQRT_3_DIV_2 + VAL_6_0;
	f.y = telemetry.ir_front + VAL_1_0; // f.x always 0
	fr.x = telemetry.ir_front_right * VAL_0_5 + VAL_2_0;
	fr.y = telemetry.ir_front_right * VAL_SQRT_3_DIV_2 + VAL_6_0;
	r.x = telemetry.ir_right * VAL_SQRT_3_DIV_2 + VAL_3_0;
	r.y = telemetry.ir_right * VAL_0_5;

	/*
	// left and right 60 degrees from center (y-axis)
	// front_left and front_right 30 degrees from center (y-axis)
	// coordinates zero between front left and right sensors
	l.x = -(telemetry.ir_left * VAL_SQRT_3_DIV_2 + VAL_3_0);
	l.y = telemetry.ir_left * VAL_0_5 - VAL_6_0;
	fl.x = -(telemetry.ir_front_left * VAL_0_5 + VAL_2_0);
	fl.y = telemetry.ir_front_left * VAL_SQRT_3_DIV_2;
	f.y = telemetry.ir_front - VAL_5_0; // f.x always 0
	fr.x = telemetry.ir_front_right * VAL_0_5 + VAL_2_0;
	fr.y = telemetry.ir_front_right * VAL_SQRT_3_DIV_2;
	r.x = telemetry.ir_right * VAL_SQRT_3_DIV_2 + VAL_3_0;
	r.y = telemetry.ir_right * VAL_0_5 - VAL_6_0;
	*/

	min_front = fl.y;
	if (min_front > f.y) {
		min_front = f.y;
	}
	if (min_front > fr.y) {
		min_front = fr.y;
	}

	// calculate weights
	inv_msum = 1 / (telemetry.ir_left + telemetry.ir_front_left + telemetry.ir_front + telemetry.ir_front_right + telemetry.ir_right);
	lm = telemetry.ir_left * inv_msum;
	flm = telemetry.ir_front_left * inv_msum;
	fm = telemetry.ir_front * inv_msum;
	frm = telemetry.ir_front_right * inv_msum;
	rm = telemetry.ir_right * inv_msum;

	// Fill missing telemetry values
	telemetry.mc.x = (lm * l.x) + (flm * fl.x) + (frm * fr.x) + (rm * r.x);
	telemetry.mc.y = (lm * l.y) + (flm * fl.y) + (fm * f.y) + (frm * fr.y) + (rm * r.y);

	/*
	 if (telemetry.ir_left < 100 && telemetry.ir_front_left < 100) {
	 telemetry.mc.x += (fl.x - l.x);
	 telemetry.mc.y += (fl.y - l.y);
	 }
	 if (telemetry.ir_right < 100 && telemetry.ir_front_right < 100) {
	 telemetry.mc.x += (fr.x - r.x);
	 telemetry.mc.y += (fr.y - r.y);
	 }
	 */

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
	fixed turn, speed_add, front_fact, angle_fact, steering_fact;

	_calc_direction(telemetry);
	maybe_stuck = (telemetry.mc_dist < 10) || (min_front < 20);
	turn = telemetry.mc_angle - 90;

	/*
	 if (turn < 0) {
	 steering_fact = (fr.x - r.x) / r.x;
	 }
	 else {
	 steering_fact = (fl.x - l.x) / l.x;
	 }
	 if (steering_fact < 0) {
	 steering_fact = 0;
	 }
	 else if (steering_fact > 1) {
	 steering_fact = 1;
	 }
	 steering_fact = 1 - steering_fact;
	 steering = int(steering_fact * 2 * turn);
	 */

	steering = 2 * int(turn);

	switch (state) {
		case STATE_NORMAL:
			// normal operation
			// steering calculations
			drive_cmd.steering_pwm = 90 - steering;

			// speed calculations
			front_fact = (telemetry.ir_front - 20) * 0.01; // correct speed by front distance
			front_fact = front_fact.clamp(VAL_0_0, VAL_1_0);
			angle_fact = (45 - turn.abs()) * VAL_1_DIV_45; // correct speed by turn angle
			angle_fact = angle_fact.clamp(VAL_0_0, VAL_1_0);
			speed_add = fixed(normal_pwm - NORMAL_FORWARD);
			speed_add = angle_fact * (front_fact * speed_add);
			drive_cmd.driving_pwm = NORMAL_FORWARD + int(speed_add);

			// for abrupt lowering of speed go to braking state
			if (speed_add < last_speed_add - 3) {
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
