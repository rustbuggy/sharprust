#include "driver.h"

#include "buggy.h"

#define STUCK_THRES 5
#define STUCK_COUNT 1000

#define BRAKING_THRES (DRIVING_MAX_FORWARD - DRIVING_NORMAL_FORWARD) / 3

static const fixed VAL_SQRT_1_DIV_2(0.70710678118654752440084436210485);
static const fixed VAL_SQRT_3_DIV_2(0.86602540378443864676372317075294);
static const fixed VAL_1_DIV_45(0.02222222222222222222222222222222);
static const fixed VAL_1_DIV_128(512, true);
static const fixed VAL_0_5(0.5);
static const fixed VAL_1_0(1);

static const fixed FRONT_MIN_FACT(0.2);
static const fixed ANGLE_MIN_FACT(0);

static const fixed SIDE_X_OFFSET(3.5);
static const fixed SIDE_Y_OFFSET(5);
static const fixed FRONT_SIDE_X_OFFSET(2.5);
static const fixed FRONT_SIDE_Y_OFFSET(1);
static const fixed FRONT_Y_OFFSET(4);

#ifdef DEBUG
#include "stdio.h"
#endif

Driver::Driver() {
  reset();
}

Driver::~Driver() {
}

void Driver::setup() {
  reset();
}

void Driver::reset() {
  state = STATE_IDLE;

  maybe_stuck = false;
  stuck_timer.stop();
  //breakout_timer.stop();

  max_forward_pwm = DRIVING_MAX_FORWARD;

  drive_cmd.steering_pwm = STEERING_NEUTRAL;
  drive_cmd.driving_pwm = DRIVING_STOP;

  steering = 0;
  last_speed_add = 0;

  tl = tfl = tf = tfr = tr = 0;
  sl = sfl = sf = sfr = sr = ssum = 0;

  accel_setpoint = 0.1f;
  pid.reset();
  driving_pwm = DRIVING_STOP;
}

void Driver::set_max_forward_pwm(uint8_t pwm) {
  max_forward_pwm = pwm;
}

bool Driver::estimate_if_stuck(bc_telemetry_packet_t& telemetry) {
  // exponential moving average of sensors
  tl = (127 * tl + telemetry.ir_left) * VAL_1_DIV_128;
  tfl = (127 * tfl + telemetry.ir_front_left) * VAL_1_DIV_128;
  tf = (127 * tf + telemetry.ir_front) * VAL_1_DIV_128;
  tfr = (127 * tfr + telemetry.ir_front_right) * VAL_1_DIV_128;
  tr = (127 * tr + telemetry.ir_right) * VAL_1_DIV_128;

  // count of how many times in a row is sensor value near its moving average
  sl = telemetry.ir_left < 90 && (tl - telemetry.ir_left).absolute() < STUCK_THRES ? sl + 1 : 0;
  sfl = telemetry.ir_front_left < 90 && (tfl - telemetry.ir_front_left).absolute() < STUCK_THRES ? sfl + 1 : 0;
  sf = telemetry.ir_front < 90 && (tf - telemetry.ir_front).absolute() < STUCK_THRES ? sf + 1 : 0;
  sfr = telemetry.ir_front_right < 90 && (tfr - telemetry.ir_front_right).absolute() < STUCK_THRES ? sfr + 1 : 0;
  sr = telemetry.ir_right < 90 && (tr - telemetry.ir_right).absolute() < STUCK_THRES ? sr + 1 : 0;

  // how many sensors have been near the same value for at least x times
  ssum = (sl > STUCK_COUNT ? 1 : 0) + (sfl > STUCK_COUNT ? 1 : 0) + (sf > STUCK_COUNT ? 1 : 0) + (sfr > STUCK_COUNT ? 1 : 0) + (sr > STUCK_COUNT ? 1 : 0);

  return (ssum > 2);
}

void Driver::calc_direction(bc_telemetry_packet_t& telemetry) {
  // left and right 45 degrees from center (y-axis)
  // front_left and front_right 30 degrees from center (y-axis)
  // coordinates zero between front wheels
  fixed a1 = telemetry.ir_left * VAL_SQRT_1_DIV_2;
  l.x = -(a1 + SIDE_X_OFFSET);
  l.y = a1 - SIDE_Y_OFFSET;
  fl.x = -(telemetry.ir_front_left * VAL_0_5 + FRONT_SIDE_X_OFFSET);
  fl.y = telemetry.ir_front_left * VAL_SQRT_3_DIV_2 + FRONT_SIDE_Y_OFFSET;
  f.y = telemetry.ir_front + FRONT_Y_OFFSET; // f.x always 0
  fr.x = telemetry.ir_front_right * VAL_0_5 + FRONT_SIDE_X_OFFSET;
  fr.y = telemetry.ir_front_right * VAL_SQRT_3_DIV_2 + FRONT_SIDE_Y_OFFSET;
  fixed a2 = telemetry.ir_right * VAL_SQRT_1_DIV_2;
  r.x = a2 + SIDE_X_OFFSET;
  r.y = a2 - SIDE_Y_OFFSET;

  // calculate weights
  if (right_default)
  {
    if (telemetry.ir_left + telemetry.ir_front_left > telemetry.ir_right + telemetry.ir_front_right + 10)
      right_default = false;
  }
  else
  {
    if (telemetry.ir_right + telemetry.ir_front_right > telemetry.ir_left + telemetry.ir_front_left + 10)
      right_default = true;
  }
  inv_msum = 1 / (telemetry.ir_left + telemetry.ir_front_left + 100 + telemetry.ir_front_right + telemetry.ir_right);
  flm = telemetry.ir_front_left * inv_msum;
  fm = telemetry.ir_front * inv_msum;
  frm = telemetry.ir_front_right * inv_msum;
  if (right_default) {
    lm = telemetry.ir_left * inv_msum;
    rm = (telemetry.ir_right + (100 - telemetry.ir_front)) * inv_msum;
  }
  else {
    lm = (telemetry.ir_left + (100 - telemetry.ir_front)) * inv_msum;
    rm = telemetry.ir_right * inv_msum;
  }

  // fill missing telemetry values
  telemetry.mc.x = (lm * l.x) + (flm * fl.x) + (frm * fr.x) + (rm * r.x);
  telemetry.mc.y = (lm * l.y) + (flm * fl.y) + (fm * f.y) + (frm * fr.y) + (rm * r.y);
  telemetry.mc_dist = telemetry.mc.get_distance();
  telemetry.mc_angle = telemetry.mc.get_deg_angle();
}

inline uint8_t Driver::clamp_steering_pwm(int32_t steer) {
  if (steer < STEERING_MIN)
  {
    return STEERING_MIN;
  } else if (steer > STEERING_MAX) {
    return STEERING_MAX;
  } else {
    return steer;
  }
}

drive_cmd_t& Driver::drive(bc_telemetry_packet_t& telemetry) {
  fixed turn, speed_add, front_fact, angle_fact, steering_fact;

  maybe_stuck = estimate_if_stuck(telemetry);

  // steering calculations
  calc_direction(telemetry);
  turn = telemetry.mc_angle - 90;
  steering = 2 * int(turn);

  // speed calculations
  front_fact = (telemetry.ir_front - 25);
  front_fact *= 0.01; // correct speed by front distance
  front_fact = front_fact.clamp(FRONT_MIN_FACT, VAL_1_0);

  angle_fact = (45 - turn.absolute()) * VAL_1_DIV_45; // correct speed by turn angle
  angle_fact = angle_fact.clamp(ANGLE_MIN_FACT, VAL_1_0);

  speed_add = fixed(max_forward_pwm - DRIVING_NORMAL_FORWARD);
  speed_add = angle_fact * (front_fact * speed_add);

  // state machine
  switch (state) {
    case STATE_NORMAL:
      drive_cmd.steering_pwm = clamp_steering_pwm(STEERING_NEUTRAL - steering);
      //driving_pwm = DRIVING_NORMAL_FORWARD + int(speed_add);

      clamp_forward_min = DRIVING_MIN_ALLOWED_FORWARD;
      clamp_forward_max = DRIVING_NORMAL_FORWARD + int(speed_add);
      driving_pwm += pid.update(accel_setpoint - telemetry.accel_x, telemetry.accel_x);
      if (driving_pwm < clamp_forward_min)
      {
        driving_pwm = clamp_forward_min;
      } else if (driving_pwm > clamp_forward_max) {
        driving_pwm = clamp_forward_max;
      }

      drive_cmd.driving_pwm = driving_pwm;

      /*
       if (breakout_timer.running()) {
       drive_cmd.driving_pwm = BREAKOUT_FORWARD;
       if (breakout_timer.triggered(telemetry.time)) {
       breakout_timer.stop();
       }
       }
       else {
       drive_cmd.driving_pwm = NORMAL_FORWARD + int(speed_add);
       }
       */

      // for abrupt lowering of speed go to braking state
      /*
      if (last_speed_add - speed_add > BRAKING_THRES) {
        stuck_timer.stop();
        last_speed_add = speed_add;
        drive_cmd.driving_pwm = DRIVING_MIN_ALLOWED_BACKWARD;
        state = STATE_BRAKING;
        break;
      }
      last_speed_add = speed_add;
      */

      // stuck countdown
      if (maybe_stuck && stuck_timer.start_or_triggered(telemetry.time, 1000, true, false)) {
        state = STATE_BACKING;
      }
      else {
        stuck_timer.stop();
      }

      break;

    case STATE_BACKING:
      drive_cmd.steering_pwm = clamp_steering_pwm(STEERING_NEUTRAL + steering);
      drive_cmd.driving_pwm = DRIVING_NORMAL_BACKWARD;

      if (/*!maybe_stuck ||*/stuck_timer.start_or_triggered(telemetry.time, 500, true, false)) {
        sl = sfl = sf = sfr = sr = ssum = 0;
        //state = STATE_BREAKOUT;
        state = STATE_NORMAL;
      }
      break;

    /*
     case STATE_BREAKOUT:
     drive_cmd.steering_pwm = clamp_steering_pwm(STEERING_NEUTRAL - steering);
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
     */

    case STATE_BRAKING:
      drive_cmd.steering_pwm = clamp_steering_pwm(STEERING_NEUTRAL - steering);
      drive_cmd.driving_pwm = DRIVING_NORMAL_BACKWARD;

      if (stuck_timer.start_or_triggered(telemetry.time, 200, true, false)) {
        state = STATE_NORMAL;
      }
      break;

    case STATE_IDLE:
    default:
      //breakout_timer.start(telemetry.time, 400);
      state = STATE_NORMAL;
      break;
  }

  Buggy::clamp_steering_and_speed(drive_cmd);

  return drive_cmd;
}
