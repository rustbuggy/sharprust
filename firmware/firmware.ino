#include <Servo.h>

#include "irsharp.h"
#include "Communication.h"
#include "Hdlc.h"

#define SHARP_MODEL_GP2Y0A21Y 1080

#define IR_LEFT             A1
#define IR_RIGHT            A0
#define IR_FRONT_LEFT       A2
#define IR_FRONT_RIGHT      A3

#define STEERING_PWM_PIN    9
#define DRIVE_PWM_PIN       11

int m_distance_left = 0;
int m_distance_right = 0;
int m_distance_front_left = 0;
int m_distance_front_right = 0;

SharpIR sharp_left(IR_LEFT, SHARP_MODEL_GP2Y0A21Y);
SharpIR sharp_right(IR_RIGHT, SHARP_MODEL_GP2Y0A21Y);
SharpIR sharp_front_left(IR_FRONT_LEFT, SHARP_MODEL_GP2Y0A21Y);
SharpIR sharp_front_right(IR_FRONT_RIGHT, SHARP_MODEL_GP2Y0A21Y);

uint8_t m_rx_buffer[40];
uint8_t m_rx_len = 0;
uint8_t m_tx_buffer[40];
uint8_t m_tx_len = 0;
HDLC hdlc(m_rx_buffer, 40);

Servo steeringservo;

void send_telemetry(uint8_t left, uint8_t right, uint8_t front_left, uint8_t front_right) {
    bc_telemetry_packet_t bc_telemetry;
    bc_telemetry.header = BC_TELEMETRY;
    bc_telemetry.ir_left = left;
    bc_telemetry.ir_right = right;
    bc_telemetry.ir_front_left = front_left;
    bc_telemetry.ir_front_right = front_right;

    m_tx_len = hdlc.encode((uint8_t*)&bc_telemetry, sizeof(bc_telemetry_packet_t), m_tx_buffer);
    Serial.write(m_tx_buffer, m_tx_len);
}

void setup() {
    Serial.begin(57600);
    pinMode(IR_LEFT, INPUT);
    pinMode(IR_RIGHT, INPUT);
    pinMode(IR_FRONT_LEFT, INPUT);
    pinMode(IR_FRONT_RIGHT, INPUT);

    steeringservo.attach(STEERING_PWM_PIN);
}

void loop() {
    while (Serial.available() > 0) {
        m_rx_len = hdlc.decode(Serial.read());

        // check if HDLC packet is received
        if (m_rx_len > 0) {
            uint8_t header = ((uint8_t*)m_rx_buffer)[0];

            if (CB_MOTOR_COMMAND == header) {
                cb_motor_command_packet_t* motor = (cb_motor_command_packet_t*)m_rx_buffer;
                //analogWrite(STEERING_PWM_PIN, motor->steering_pwm);
                steeringservo.write(motor->steering_pwm);
                analogWrite(DRIVE_PWM_PIN, motor->drive_pwm);
            }
        }
    }

    m_distance_left = sharp_left.distance();
    m_distance_right = sharp_right.distance();
    m_distance_front_left = sharp_front_left.distance();
    m_distance_front_right = sharp_front_right.distance();

    send_telemetry(m_distance_left, m_distance_right, m_distance_front_left, m_distance_front_right);

    //Serial.print("left: ");
    //Serial.println(m_distance_left);
}
