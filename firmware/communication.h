#ifndef __COMMUNICATION__H
#define __COMMUNICATION__H

#include "types.h"

// bc means from Buggy to Computer
// cb means from Computer tp Buggy
enum {
	BC_TELEMETRY = 0x01,

	CB_MOTOR_COMMAND = 0x02,
};

typedef struct __attribute__((packed)) {
	uint8_t header; // BC_TELEMETRY
	uint32_t time;
	fixed_t ir_left;
	fixed_t ir_right;
	fixed_t ir_front_left;
	fixed_t ir_front_right;
	fixed_t ir_front;
	fixed_t mc_x;
	fixed_t mc_y;
	fixed_t mc_dist;
	fixed_t mc_angle;

	bool change_steering;
	int32_t steering_pwm;
	bool change_driving;
	int32_t driving_pwm;
} bc_telemetry_packet_t;

typedef struct __attribute__((packed)) {
	uint8_t header; // CB_MOTOR_COMMAND
	uint8_t automatic;
	uint8_t steering_pwm;
	uint8_t drive_pwm;
} cb_motor_command_packet_t;

#endif
