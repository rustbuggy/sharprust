
#ifndef __COMMUNICAYION__H
#define __COMMUNICAYION__H

// bc means from Buggy to Computer
// cb means from Computer tp Buggy

enum {
	BC_TELEMETRY = 0x00,
};

typedef struct {
	uint8_t header;
	uint8_t ir_left;
	uint8_t ir_right;
	uint8_t ir_front_left;
	uint8_t ir_front_right;
} bc_telemetry_packet_t __attribute__((packed));

#endif