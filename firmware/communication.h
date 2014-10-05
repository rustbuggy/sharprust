
#ifndef __COMMUNICAYION__H
#define __COMMUNICAYION__H

// bc means from Buggy to Computer
// cb means from Computer tp Buggy

enum {
	BC_TELEMETRY = 0x00,

	CB_MOTOR_COMMAND = 0x01,
};

typedef struct {
	uint8_t header; // BC_TELEMETRY
	uint8_t ir_left;
	uint8_t ir_right;
	uint8_t ir_front_left;
	uint8_t ir_front_right;
} bc_telemetry_packet_t __attribute__((packed));

typedef struct {
	uint8_t header; // CB_MOTOR_COMMAND
	uint8_t steering_pwm;
	uint8_t drive_pwm;
} cb_motor_command_packet_t __attribute__((packed));

#endif