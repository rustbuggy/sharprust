#include "communicator.h"

#include "buggy.h"

#define TELEMETRY_SEND_INTERVAL_MS      20

Communicator::Communicator() :
    rx_len(0), tx_len(0), hdlc(rx_buffer, 255) {
}

Communicator::~Communicator() {
}

void Communicator::setup() {
  SERIALDEV.begin(57600);
}

void Communicator::send_telemetry(bc_telemetry_packet_t& telemetry) {
  if (send_telemetry_timer.start_or_triggered(telemetry.time, TELEMETRY_SEND_INTERVAL_MS, false, true)) {
    tx_len = hdlc.encode((uint8_t*) &telemetry, sizeof(bc_telemetry_packet_t), tx_buffer);
    SERIALDEV.write(tx_buffer, tx_len);
  }
}

drive_cmd_t& Communicator::read_command(bc_telemetry_packet_t& telemetry) {
  drive_cmd.received = false;

  while (SERIALDEV.available() > 0) {
    rx_len = hdlc.decode(SERIALDEV.read());

    // check if HDLC packet is received
    if (rx_len > 0) {
      uint8_t header = ((uint8_t*) rx_buffer)[0];

      if (CB_MOTOR_COMMAND == header) {
        cb_motor_command_packet_t* motor = (cb_motor_command_packet_t*) rx_buffer;
        drive_cmd.received = true;
        drive_cmd.automatic = motor->automatic;
        drive_cmd.steering_pwm = motor->steering_pwm;
        drive_cmd.driving_pwm = motor->driving_pwm;
      }
    }
  }

  Buggy::clamp_steering_and_speed(drive_cmd);

  return drive_cmd;
}
