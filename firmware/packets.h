#ifndef __PACKETS__H
#define __PACKETS__H

#include "types.h"

#define STEERING_MIN                    30
#define STEERING_NEUTRAL                90
#define STEERING_MAX                    150

#define DRIVING_STOP                    90
#define DRIVING_NORMAL_FORWARD          105//100
#define DRIVING_MAX_NORMAL_FORWARD      111//106
#define DRIVING_MAX_FORWARD             125//120
#define DRIVING_BREAKOUT_FORWARD        115//110
#define DRIVING_MIN_ALLOWED_FORWARD     105//100
#define DRIVING_MAX_ALLOWED_FORWARD     DRIVING_BREAKOUT_FORWARD

#define DRIVING_NORMAL_BACKWARD         30
#define DRIVING_MAX_ALLOWED_BACKWARD    30

// bc means from Buggy to Computer
// cb means from Computer to Buggy
enum {
  BC_TELEMETRY = 0x01, CB_MOTOR_COMMAND = 0x02
};

typedef struct __attribute__((packed)) bc_telemetry_packet_t {
public:
  uint8_t header; // BC_TELEMETRY
  uint32_t time;
  uint32_t cycles;
  fixed ir_left;
  fixed ir_right;
  fixed ir_front_left;
  fixed ir_front_right;
  fixed ir_front;
  point_t mc;
  fixed mc_dist;
  fixed mc_angle;

  float accel_x;
  float accel_y;
  float accel_z;
  float speed_x;
  float speed_y;
  float speed_z;

  uint8_t automatic;
  uint8_t steering_pwm;
  uint8_t driving_pwm;

  uint16_t battery;
  uint8_t state;

  bc_telemetry_packet_t() {
    header = BC_TELEMETRY;
    time = 0;
    cycles = 0;
    ir_left = 0;
    ir_right = 0;
    ir_front_left = 0;
    ir_front_right = 0;
    ir_front = 0;
    mc_dist = 0;
    mc_angle = 0;

    accel_x = 0.0f;
    accel_y = 0.0f;
    accel_z = 0.0f;
    speed_x = 0.0f;
    speed_y = 0.0f;
    speed_z = 0.0f;

    automatic = 0; // false
    steering_pwm = STEERING_NEUTRAL;
    driving_pwm = DRIVING_STOP;

    battery = 0;
    state = 0;
  }
} bc_telemetry_packet_t;

typedef struct __attribute__((packed)) cb_motor_command_packet_t {
public:
  uint8_t header; // CB_MOTOR_COMMAND
  uint8_t automatic;
  uint8_t steering_pwm;
  uint8_t driving_pwm;

  cb_motor_command_packet_t() {
    header = CB_MOTOR_COMMAND;
    automatic = 0; // false
    steering_pwm = STEERING_NEUTRAL;
    driving_pwm = DRIVING_STOP;
  }
} cb_motor_command_packet_t;

typedef struct __attribute__((packed)) drive_cmd_t {
public:
  bool received;
  uint8_t automatic;
  uint8_t steering_pwm;
  uint8_t driving_pwm;

  drive_cmd_t() {
    received = false;
    automatic = 0; // false
    steering_pwm = STEERING_NEUTRAL;
    driving_pwm = DRIVING_STOP;
  }
} drive_cmd_t;

#endif //  __PACKETS__H
