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

#define START_BUTTON_DEBOUNCE_TIME_MS       250
#define START_DELAY_MS                      5050

#define BATTERY_CHECK_INTERVAL_MS           1000
#define BATTERY_LOW_MILLI_VOLTAGE           7500

#define TEENSY_LED          13 // (LED)
#define START_BUTTON        14 // A0
#define GREEN_LED           16 // A2
#define RED_LED             15 // A1

#define BATTERY             A4

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
uint32_t m_last_telemetry_time = 0;

bool m_button_press = false;
bool m_button_active = true;
uint32_t m_last_button_time = 0;
bool m_countdown = false;
uint32_t m_countdown_start_time = 0;

uint32_t m_time;
uint32_t m_last_battery_check_time = 0;

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
	pinMode(GREEN_LED, OUTPUT);
	pinMode(RED_LED, OUTPUT);
	pinMode(BATTERY, INPUT);

	pinMode(START_BUTTON, INPUT_PULLUP);
	attachInterrupt(START_BUTTON, button_service, RISING);

	telemetry.header = BC_TELEMETRY;
}

void button_service() {
	if (m_button_active) {
		m_button_press = true;
		m_button_active = false;
		m_last_button_time = millis();
	}
}

void toggle_led(uint8_t led) {
	static bool ledon = true;
	if (ledon) {
		ledon = false;
		digitalWrite(led, HIGH);
	}
	else {
		ledon = true;
		digitalWrite(led, LOW);
	}
}

void blink_automatic_mode() {
	static uint32_t last_blink;
	if (millis() - last_blink > 125) {
		toggle_led(TEENSY_LED);
		last_blink = millis();
	}
}

uint32_t battery_voltage() {
	const uint32_t vref = 3600000; // 3.6 V
	const uint32_t voltage_divider = 2710; // divides 2.7 times
	return vref/1024 * analogRead(BATTERY) * (uint64_t)voltage_divider/1000000;
}

void loop() {
	m_time = millis();

	if (m_button_press) {
		m_button_press = false;

		m_countdown = true;
		m_countdown_start_time = m_time;

		digitalWrite(TEENSY_LED, HIGH);
		digitalWrite(GREEN_LED, HIGH);
	}

	// handle button debounce
	if (!m_button_active && (m_time > m_last_button_time +  START_BUTTON_DEBOUNCE_TIME_MS)) {
		m_button_active = true;
	}

	// ready to race!?
	if (m_countdown && (m_time > m_countdown_start_time +  START_DELAY_MS)) {
		m_button_active = true;
		m_countdown = false;

		driver.reset();
		m_automatic = true;
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

	telemetry.time = m_time;
	telemetry.ir_left = sharp_left.distance();
	telemetry.ir_right = sharp_right.distance();
	telemetry.ir_front_left = sharp_front_left.distance();
	telemetry.ir_front_right = sharp_front_right.distance();
	telemetry.ir_front = sharp_front.distance();

	drive_cmd_t& drive_cmd = driver.drive(telemetry);
	if (m_automatic) {
		steeringservo.write(drive_cmd.steering_pwm);
		drivingservo.write(drive_cmd.driving_pwm);
		blink_automatic_mode();
	}

	if (m_time > m_last_telemetry_time + 20) {
		send_telemetry();
		m_last_telemetry_time = m_time;
	}

	if (m_time > m_last_battery_check_time + BATTERY_CHECK_INTERVAL_MS) {
		m_last_battery_check_time = m_time;
		if (battery_voltage() < BATTERY_LOW_MILLI_VOLTAGE) {
			digitalWrite(RED_LED, HIGH);
		}
		else {
			digitalWrite(RED_LED, LOW);
		}
	}
}

