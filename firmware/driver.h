#ifndef __DRIVER__H
#define __DRIVER__H

#include "types.h"
#include "packets.h"
#include "timer.h"
#include "pid.h"

typedef enum driver_states_t {
  STATE_IDLE = 1, STATE_NORMAL, STATE_BACKING, /*STATE_BREAKOUT,*/STATE_BRAKING
} driver_states_t;

// Drive towards mass center of estimated IR reflection points
class Driver {
private:
  driver_states_t state;

  bool maybe_stuck;
  timer stuck_timer;
  //timer breakout_timer;
  fixed min_front;

  uint8_t max_forward_pwm;

  point_t l, fl, f, fr, r;

  int32_t steering;

  timer last_speed_add_timer;
  fixed last_speed_add;

  fixed tl, tfl, tf, tfr, tr;
  uint32_t sl, sfl, sf, sfr, sr, ssum;
  fixed lm, flm, fm, frm, rm, inv_msum;

  float accel_setpoint;
  PID pid;
  float driving_pwm;
  uint8_t clamp_forward_max, clamp_forward_min;

  bool right_default;

  bool estimate_if_stuck(bc_telemetry_packet_t& telemetry);
  void calc_direction(bc_telemetry_packet_t& telemetry);
  inline uint8_t clamp_steering_pwm(int32_t steer);

public:
  drive_cmd_t drive_cmd;

  Driver();
  virtual ~Driver();

  void setup();
  void reset();
  drive_cmd_t& drive(bc_telemetry_packet_t& telemetry);
  void set_max_forward_pwm(uint8_t pwm);
};

#endif // __DRIVER__H
