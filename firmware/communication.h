
#ifndef __COMMUNICAYION__H
#define __COMMUNICAYION__H

// bc means from Buggy to Computer
// cb means from Computer tp Buggy

enum {
	BC_TELEMETRY = 0x00,

	CB_MOTOR_COMMAND = 0x01,
};

typedef struct bc_telemetry_packet_t {
	uint8_t header; // BC_TELEMETRY
	uint8_t ir_left;
	uint8_t ir_left_ewma;
	uint8_t ir_right;
	uint8_t ir_front_left;
	uint8_t ir_front_right;
        int32_t mc_x;
        int32_t mc_y;

        bool changeSteering;
        int32_t steeringPwm;
        bool changeDriving;
        int32_t drivingPwm;
} bc_telemetry_packet_t __attribute__((packed));

typedef struct cb_motor_command_packet_t {
	uint8_t header; // CB_MOTOR_COMMAND
	uint8_t steering_pwm;
	uint8_t drive_pwm;
} cb_motor_command_packet_t __attribute__((packed));

#endif
