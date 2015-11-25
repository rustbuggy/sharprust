#ifndef BUGGY_H_
#define BUGGY_H_

#include <Servo.h>
#include <ADC.h>

#include "irsharp.h"
#include "packets.h"
#include "timer.h"
#include "accel.h"

#define BATTERY_CHECK_INTERVAL_MS       5000
#define BATTERY_LOW_MILLI_VOLTAGE       7700

#define START_BUTTON_DEBOUNCE_TIME_MS   250
#define START_DELAY_MS                  5050

#define AUTO_BLINK_INTERVAL_MS          125

#define WINDOW_SIZE         4
#define INFINITY_VARIANCE   fixed(49)
#define INFINITY_DISTANCE   fixed(100)

class Buggy {
private:
  uint32_t setup_time;

  ADC* adc;

  /*
  IRSharp sharp_left;
  IRSharp sharp_right;
  IRSharp sharp_front_left;
  IRSharp sharp_front_right;
  IRSharp sharp_front;
  */

  Accel accel;

  uint8_t read_ind;
  fixed reads[5][WINDOW_SIZE];
  fixed average[5];
  fixed variance[5];

  uint8_t steering_pwm;
  uint8_t driving_pwm;

  Servo steeringservo;
  Servo drivingservo;

  timer ir_check_timer;
  timer battery_check_timer;

  timer blink_timer;
  bool led_state;

  timer countdown_timer;

  uint16_t battery_voltage();
  void auto_mode_blink(uint32_t time);

public:
  static void clamp_steering_and_speed(drive_cmd_t& drive_cmd);

  Buggy();
  virtual ~Buggy();

  void setup();
  void sense(bc_telemetry_packet_t& telemetry);
  void act(bc_telemetry_packet_t& telemetry);
};

#endif // BUGGY_H_
