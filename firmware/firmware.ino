#include <Servo.h>

#include "irsharp.h"
#include "Communication.h"
#include "Hdlc.h"
#include "mcdriver.h"

#define IR_LEFT             A1
#define IR_RIGHT            A0
#define IR_FRONT_LEFT       A2
#define IR_FRONT_RIGHT      A3

#define STEERING_PWM_PIN    9
#define DRIVE_PWM_PIN       11

bc_telemetry_packet_t telemetry;

IRSharp sharp_left(IR_LEFT);
IRSharp sharp_right(IR_RIGHT);
IRSharp sharp_front_left(IR_FRONT_LEFT);
IRSharp sharp_front_right(IR_FRONT_RIGHT);

uint8_t m_rx_buffer[64];
uint8_t m_rx_len = 0;
uint8_t m_tx_buffer[64];
uint8_t m_tx_len = 0;
HDLC hdlc(m_rx_buffer, 64);

Servo steeringservo;
Servo drivingservo;

bool automatic = false;

uint32_t last_time = 0;

MCDriver driver;

void send_telemetry() {
  m_tx_len = hdlc.encode((uint8_t*)&telemetry, sizeof(bc_telemetry_packet_t), m_tx_buffer);
  Serial.write(m_tx_buffer, m_tx_len);
}

void setup() {
  Serial.begin(57600);

  steeringservo.attach(STEERING_PWM_PIN);
  drivingservo.attach(DRIVE_PWM_PIN);

  analogReference(DEFAULT);

  telemetry.header = BC_TELEMETRY;
}

void loop() {
  while (Serial.available() > 0)
  {
    m_rx_len = hdlc.decode(Serial.read());

    // check if HDLC packet is received
    if (m_rx_len > 0)
    {
      uint8_t header = ((uint8_t*)m_rx_buffer)[0];

      if (CB_MOTOR_COMMAND == header)
      {
        cb_motor_command_packet_t* motor = (cb_motor_command_packet_t*)m_rx_buffer;
        automatic = motor->automatic != 0;
        if (!automatic)
        {
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

  drive_cmd_t& driveCmd = driver.drive(telemetry);
  if (automatic)
  {
    if (driveCmd.changeSteering)
    {
      steeringservo.write(driveCmd.steeringPwm);
    }
    if (driveCmd.changeDriving)
    {
      drivingservo.write(driveCmd.drivingPwm);
    }
  }

  if (telemetry.time > last_time + 20)
  {
    send_telemetry();
    last_time = telemetry.time;
  }

  //delay(10);

  //Serial.print("left: ");
  //Serial.println(m_distance_left);
}


