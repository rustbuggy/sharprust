#ifndef __DRIVER__H
#define __DRIVER__H

#include "types.h"
#include "packets.h"
#include "timer.h"
#include "pid.h"

typedef enum driver_states_t {
  STATE_IDLE = 0, STATE_NORMAL, STATE_START_BACKING, STATE_BACKING, STATE_BREAKOUT, STATE_BRAKING
} driver_states_t;

// Drive towards mass center of estimated IR reflection points
class Driver {
  private:
    driver_states_t state, last_state;

    bool maybe_stuck;
    timer stuck_timer;
    timer start_backing_timer;
    timer backing_timer;
    timer breakout_timer;
    timer braking_timer;
    fixed min_front;

    uint8_t max_forward_pwm;

    point_t l, fl, f, fr, r;

    fixed speed_add;
    int32_t steering;

    fixed tl, tfl, tf, tfr, tr;
    uint32_t sl, sfl, sf, sfr, sr, ssum;
    fixed lm, flm, fm, frm, rm, inv_msum;

    float accel_setpoint;
    PID pid;
    float driving_pwm, last_driving_pwm;
    uint8_t clamp_forward_max, clamp_forward_min;

    bool right_default;

    bool estimate_if_stuck(bc_telemetry_packet_t& telemetry);
    void calc_direction(bc_telemetry_packet_t& telemetry);
    inline uint8_t clamp_steering_pwm(int32_t steer);

    driver_states_t normal(bc_telemetry_packet_t& telemetry);
    driver_states_t start_backing(bc_telemetry_packet_t& telemetry);
    driver_states_t backing(bc_telemetry_packet_t& telemetry);
    driver_states_t breakout(bc_telemetry_packet_t& telemetry);
    driver_states_t braking(bc_telemetry_packet_t& telemetry);

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
