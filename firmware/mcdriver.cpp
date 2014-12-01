#include "mcdriver.h"

#define STEERING_MAX 150
#define STEERING_MIN 30
#define STEERING_NEUTRAL 90
#define STOP 95
#define NORMAL_FORWARD 106
#define MAX_FORWARD 112
#define BREAKOUT_FORWARD 115
#define MAX_ALLOWED_FORWARD 115
#define NORMAL_BACKWARD 40
#define MIN_ALLOWED_BACKWARD 40

#define STUCK_THRES 12
#define STUCK_COUNT 1000

#define BRAKING_THRES ((MAX_FORWARD - NORMAL_FORWARD) / 4 + 0)//1)

static const fixed VAL_SQRT_1_DIV_2(0.70710678118654752440084436210485);
static const fixed VAL_SQRT_3_DIV_2(0.86602540378443864676372317075294);
static const fixed VAL_1_DIV_45(0.02222222222222222222222222222222);

static const fixed VAL_1_DIV_128(512, true);

static const fixed VAL_FRONT_MIN_FACT(0.2);

static const fixed VAL_0_0(0);
static const fixed VAL_0_5(0.5);
static const fixed VAL_1_0(1);
//static const fixed VAL_2_0(2);
static const fixed VAL_2_5(2.5);
//static const fixed VAL_3_0(3);
static const fixed VAL_3_5(3.5);
static const fixed VAL_4_0(4);
static const fixed VAL_5_0(5);
//static const fixed VAL_6_0(6);

static const fixed FRONT_MIN_STUCK(13);

#ifdef DEBUG
#include "stdio.h"
#endif

MCDriver::MCDriver() {
	reset();
}

void MCDriver::reset() {
	state = STATE_IDLE;

	maybe_stuck = false;
	stuck_timer.stop();
	breakout_timer.stop();

	normal_pwm = MAX_FORWARD;

	drive_cmd.steering_pwm = STEERING_NEUTRAL;
	drive_cmd.driving_pwm = STOP;

	steering = 0;
	last_speed_add_timer.start(1, 1);
	last_speed_add = 0;

	tl = 0;
	tfl = 0;
	tf = 0;
	tfr = 0;
	tr = 0;

	sl, sfl, sf, sfr, sr, ssum = 0;
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
	if (drive_cmd.driving_pwm > MAX_ALLOWED_FORWARD) {
		drive_cmd.driving_pwm = MAX_ALLOWED_FORWARD;
	}
	else if (drive_cmd.driving_pwm < MIN_ALLOWED_BACKWARD) {
		drive_cmd.driving_pwm = MIN_ALLOWED_BACKWARD;
	}

	// Fill missing telemetry values
	telemetry.steering_pwm = drive_cmd.steering_pwm;
	telemetry.driving_pwm = drive_cmd.driving_pwm;
}

void MCDriver::_calc_direction(bc_telemetry_packet_t& telemetry) {
	tl = (127 * tl + telemetry.ir_left) * VAL_1_DIV_128;
	tfl = (127 * tfl + telemetry.ir_front_left) * VAL_1_DIV_128;
	tf = (127 * tf + telemetry.ir_front) * VAL_1_DIV_128;
	tfr = (127 * tfr + telemetry.ir_front_right) * VAL_1_DIV_128;
	tr = (127 * tr + telemetry.ir_right) * VAL_1_DIV_128;

	sl = (tl - telemetry.ir_left).abs() < STUCK_THRES ? sl + 1 : 0;
	sfl = (tfl - telemetry.ir_front_left).abs() < STUCK_THRES ? sfl + 1 : 0;
	sf = (tf - telemetry.ir_front).abs() < STUCK_THRES ? sf + 1 : 0;
	sfr = (tfr - telemetry.ir_front_right).abs() < STUCK_THRES ? sfr + 1 : 0;
	sr = (tr - telemetry.ir_right).abs() < STUCK_THRES ? sr + 1 : 0;
	ssum = (sl > STUCK_COUNT ? 1 : 0) + (sfl > STUCK_COUNT ? 1 : 0) + (sf > STUCK_COUNT ? 1 : 0) + (sfr > STUCK_COUNT ? 1 : 0)
			+ (sr > STUCK_COUNT ? 1 : 0);

	/*
	// left and right 45 degrees from center (y-axis)
	// front_left and front_right 30 degrees from center (y-axis)
	// coordinates zero between left and right sensor
	fixed a1 = telemetry.ir_left * VAL_SQRT_1_DIV_2;
	l.x = -(a1 + VAL_3_5);
	l.y = a1;
	fl.x = -(telemetry.ir_front_left * VAL_0_5 + VAL_2_5);
	fl.y = telemetry.ir_front_left * VAL_SQRT_3_DIV_2 + VAL_6_0;
	f.y = telemetry.ir_front + VAL_1_0; // f.x always 0
	fr.x = telemetry.ir_front_right * VAL_0_5 + VAL_2_5;
	fr.y = telemetry.ir_front_right * VAL_SQRT_3_DIV_2 + VAL_6_0;
	fixed a2 = telemetry.ir_right * VAL_SQRT_1_DIV_2;
	r.x = a2 + VAL_3_5;
	r.y = a2;
	*/

	// left and right 45 degrees from center (y-axis)
	// front_left and front_right 30 degrees from center (y-axis)
	// coordinates zero between front wheels
	fixed a1 = telemetry.ir_left * VAL_SQRT_1_DIV_2;
	l.x = -(a1 + VAL_3_5);
	l.y = a1 - VAL_5_0;
	fl.x = -(telemetry.ir_front_left * VAL_0_5 + VAL_2_5);
	fl.y = telemetry.ir_front_left * VAL_SQRT_3_DIV_2 + VAL_1_0;
	f.y = telemetry.ir_front + VAL_4_0; // f.x always 0
	fr.x = telemetry.ir_front_right * VAL_0_5 + VAL_2_5;
	fr.y = telemetry.ir_front_right * VAL_SQRT_3_DIV_2 + VAL_1_0;
	fixed a2 = telemetry.ir_right * VAL_SQRT_1_DIV_2;
	r.x = a2 + VAL_3_5;
	r.y = a2 - VAL_5_0;

	/*
	 min_front = l.y;
	 if (min_front > fl.y) {
	 min_front = fl.y;
	 }
	 if (min_front > f.y) {
	 min_front = f.y;
	 }
	 if (min_front > fr.y) {
	 min_front = fr.y;
	 }
	 if (min_front > r.y) {
	 min_front = r.y;
	 }
	 */

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
	//telemetry.mc.y = (flm * fl.y) + (fm * f.y) + (frm * fr.y);

	/*
	 fixed inv_wsum = 1 / (telemetry.ir_left + telemetry.ir_front_left + telemetry.ir_front + telemetry.ir_front_right + telemetry.ir_right);
	 telemetry.mc.x = (telemetry.ir_left * l.x + telemetry.ir_front_left * fl.x + telemetry.ir_front_right * fr.x + telemetry.ir_right * r.x)
	 * inv_wsum;
	 telemetry.mc.y = (telemetry.ir_front_left * fl.y + telemetry.ir_front * f.y + telemetry.ir_front_right * fr.y) * inv_wsum;
	 */

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

	//maybe_stuck = (telemetry.mc_dist < 10) || (min_front < 20);
	//maybe_stuck = (min_front < 10);
	maybe_stuck = (ssum > 2) || (telemetry.ir_front < FRONT_MIN_STUCK);

	// steering calculations
	turn = telemetry.mc_angle - 90;

	/*
	if (turn < 0) {
		steering_fact = (fr.x - r.x) / r.x;
	}
	else {
		steering_fact = (fl.x - l.x) / l.x;
	}
	steering_fact.clamp(VAL_0_0, VAL_1_0);
	steering_fact = 1 - steering_fact;
	steering = int(steering_fact * 2 * turn);
	*/

	steering = 2 * int(turn);

	// speed calculations
	front_fact = (telemetry.ir_front - 25) * 0.01; // correct speed by front distance
	front_fact = front_fact.clamp(VAL_FRONT_MIN_FACT, VAL_1_0);
	angle_fact = (30 - turn.abs()) * VAL_1_DIV_45; // correct speed by turn angle
	angle_fact = angle_fact.clamp(VAL_0_0, VAL_1_0);
	speed_add = fixed(normal_pwm - NORMAL_FORWARD);
	speed_add = angle_fact * (front_fact * speed_add);

	switch (state) {
		case STATE_NORMAL:
			// normal operation
			drive_cmd.steering_pwm = 90 - steering;
			if (breakout_timer.running()) {
				drive_cmd.driving_pwm = BREAKOUT_FORWARD;
				if (breakout_timer.triggered(telemetry.time)) {
					breakout_timer.stop();
				}
			}
			else {
				drive_cmd.driving_pwm = NORMAL_FORWARD + int(speed_add);
			}

			// for abrupt lowering of speed go to braking state
			if (last_speed_add - speed_add > BRAKING_THRES) {
				stuck_timer.stop();
				last_speed_add = speed_add;
				drive_cmd.driving_pwm = MIN_ALLOWED_BACKWARD;
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
			if (turn.abs() < 10)
			{
				drive_cmd.steering_pwm = 90 + steering + 10;
			} else {
				drive_cmd.steering_pwm = 90 + steering;
			}
			drive_cmd.driving_pwm = NORMAL_BACKWARD;

			if (!stuck_timer.running()) {
				stuck_timer.start(telemetry.time, 500);
			}
			if (/*!maybe_stuck ||*/ stuck_timer.triggered(telemetry.time)) {
				stuck_timer.stop();
				state = STATE_BREAKOUT;
			}
			break;

		case STATE_BREAKOUT:
			drive_cmd.steering_pwm = 90 - steering;
			drive_cmd.driving_pwm = BREAKOUT_FORWARD;

			if (!stuck_timer.running()) {
				stuck_timer.start(telemetry.time, 1000);
			}
			if (!maybe_stuck) {
				stuck_timer.stop();
				breakout_timer.start(telemetry.time, 200);
				state = STATE_NORMAL;
			}
			else if (stuck_timer.triggered(telemetry.time)) {
				stuck_timer.stop();
				state = STATE_BACKING;
			}
			break;

		case STATE_BRAKING:
			drive_cmd.steering_pwm = 90 - steering;
			drive_cmd.driving_pwm = NORMAL_BACKWARD;

			if (!stuck_timer.running()) {
				stuck_timer.start(telemetry.time, 200);
			}
			else if (stuck_timer.triggered(telemetry.time)) {
				stuck_timer.stop();
				state = STATE_NORMAL;
			}
			break;

		case STATE_IDLE:
		default:
			breakout_timer.start(telemetry.time, 400);
			state = STATE_NORMAL;
			break;
	}

	_clamp_steering_and_speed(telemetry);

	telemetry.battery = state;

	return drive_cmd;
}
