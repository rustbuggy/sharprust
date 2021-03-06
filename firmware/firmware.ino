#include <Servo.h>
#include <SPI.h>
#include <EEPROM.h>

#include <ADC.h>

#include "buggy.h"
#include "communicator.h"
#include "driver.h"

bc_telemetry_packet_t telemetry; // Telemetry packet, but also main shared information structure

Buggy buggy; // Models the actual buggy, sensors and actuators
Communicator comm; // Manual control and telemetry output
Driver driver; // The "intelligence"

uint8_t was_automatic;

void setup() {
  delay(1000);

  comm.setup();
  buggy.setup();
  driver.setup();
}

void loop() {
  was_automatic = telemetry.automatic;

  buggy.sense(telemetry);

  drive_cmd_t& manual_cmd = comm.read_command(telemetry);

  if (manual_cmd.received) {
    telemetry.automatic = manual_cmd.automatic;
  }

  if (!was_automatic && telemetry.automatic) {
    driver.reset(); // on change to automatic start from scratch
    telemetry.speed_x = 0.0f;
    telemetry.speed_y = 0.0f;
    telemetry.speed_z = 0.0f;
  }

  if (manual_cmd.received) {
    if (telemetry.automatic) {
      //driver.set_max_forward_pwm(manual_cmd.driving_pwm);
    }
    else {
      telemetry.steering_pwm = manual_cmd.steering_pwm;
      telemetry.driving_pwm = manual_cmd.driving_pwm;
    }
  }

  drive_cmd_t& auto_cmd = driver.drive(telemetry);

  if (telemetry.automatic) {
    telemetry.steering_pwm = auto_cmd.steering_pwm;
    telemetry.driving_pwm = auto_cmd.driving_pwm;
  }

  buggy.act(telemetry);

  comm.send_telemetry(telemetry);
}

