#include <Servo.h>

#include "irsharp.h"
#include "communication.h"
#include "Hdlc.h"
#include "mcdriver.h"

// comment out for USB serial
#define USE_XBEE_FOR_TELEMETRY

#ifdef USE_XBEE_FOR_TELEMETRY
#define SERIALDEV Serial3
#else
#define SERIALDEV Serial
#endif

#define START_BUTTON_DEBOUNCE_TIME_MS        250

#define TEENSY_LED          13
#define START_BUTTON        14

#define IR_LEFT             A9
#define IR_FRONT_LEFT       A8
#define IR_FRONT            A5
#define IR_FRONT_RIGHT      A7
#define IR_RIGHT            A6

#define STEERING_PWM_PIN    4
#define DRIVE_PWM_PIN       3

bc_telemetry_packet_t telemetry;

IRSharp sharp_left(IR_LEFT);
IRSharp sharp_right(IR_RIGHT);
IRSharp sharp_front_left(IR_FRONT_LEFT);
IRSharp sharp_front_right(IR_FRONT_RIGHT);
IRSharp sharp_front(IR_FRONT);

uint8_t m_rx_buffer[255];
uint8_t m_rx_len = 0;
uint8_t m_tx_buffer[255];
uint8_t m_tx_len = 0;
HDLC hdlc(m_rx_buffer, 255);

Servo steeringservo;
Servo drivingservo;

bool m_automatic = false;
uint32_t last_time = 0;

bool m_button_press = false;
bool m_button_active = false;
bool m_last_button_time = 0;

MCDriver driver;

void send_telemetry() {
	m_tx_len = hdlc.encode((uint8_t*) &telemetry, sizeof(bc_telemetry_packet_t), m_tx_buffer);
	SERIALDEV.write(m_tx_buffer, m_tx_len);
}

void setup() {
	SERIALDEV.begin(57600);

	steeringservo.attach(STEERING_PWM_PIN);
	drivingservo.attach(DRIVE_PWM_PIN);

	analogReference (DEFAULT);
	analogReadAveraging(16);
	analogReadResolution(10);
	pinMode(TEENSY_LED, OUTPUT);

	pinMode(START_BUTTON, INPUT_PULLUP);
	attachInterrupt(START_BUTTON, button_service, FALLING);

	telemetry.header = BC_TELEMETRY;
}

void button_service() {
	if (m_button_active) {
		m_button_press = true;
		m_button_active = false;
		m_last_button_time = millis();
	}
}

void toggle_led() {
	static bool ledon = true;
	if (ledon) {
		ledon = false;
		digitalWrite(TEENSY_LED, HIGH);
	}
	else {
		ledon = true;
		digitalWrite(TEENSY_LED, LOW);
	}
}

void loop() {
	if (m_button_press) {
		m_button_press = false;
		m_automatic = true;
	}

	// handle debounce
	if (!m_button_active && ((millis() - m_last_button_time) > START_BUTTON_DEBOUNCE_TIME_MS)) {
		m_button_active = true;
	}

	while (SERIALDEV.available() > 0) {
		m_rx_len = hdlc.decode(SERIALDEV.read());

		// check if HDLC packet is received
		if (m_rx_len > 0) {
			uint8_t header = ((uint8_t*) m_rx_buffer)[0];

			if (CB_MOTOR_COMMAND == header) {
				cb_motor_command_packet_t* motor = (cb_motor_command_packet_t*) m_rx_buffer;
				m_automatic = motor->automatic;
				if (m_automatic) {
					driver.set_drive_pwm(motor->drive_pwm);
				}
				else {
					steeringservo.write(motor->steering_pwm);
					drivingservo.write(motor->drive_pwm);
				}
			}
		}
	}

	telemetry.time = millis();
	telemetry.ir_left = sharp_left.distance();
	telemetry.ir_right = sharp_right.distance();
	telemetry.ir_front_left = sharp_front_left.distance();
	telemetry.ir_front_right = sharp_front_right.distance();
	telemetry.ir_front = sharp_front.distance();

	drive_cmd_t& drive_cmd = driver.drive(telemetry);
	if (m_automatic) {
		steeringservo.write(drive_cmd.steering_pwm);
		drivingservo.write(drive_cmd.driving_pwm);
		toggle_led();
	}

	if (telemetry.time > last_time + 20) {
		send_telemetry();
		last_time = telemetry.time;
	}
}

