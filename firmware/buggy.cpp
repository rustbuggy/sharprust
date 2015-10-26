#include "buggy.h"

#include "pins.h"
#include "lookups.h"

volatile bool button_pressed = false;
volatile uint32_t last_button_time = 0;

void button_service() {
  uint32_t curr_button_time = millis();
  if (curr_button_time > last_button_time + START_BUTTON_DEBOUNCE_TIME_MS) {
    last_button_time = curr_button_time;
    button_pressed = true;
  }
}

void Buggy::clamp_steering_and_speed(drive_cmd_t& cmd) {
  // steering
  if (cmd.steering_pwm > STEERING_MAX) {
    cmd.steering_pwm = STEERING_MAX;
  }
  else if (cmd.steering_pwm < STEERING_MIN) {
    cmd.steering_pwm = STEERING_MIN;
  }

  // speed
  if (cmd.driving_pwm > MAX_ALLOWED_FORWARD) {
    cmd.driving_pwm = MAX_ALLOWED_FORWARD;
  }
  else if (cmd.driving_pwm < MIN_ALLOWED_BACKWARD) {
    cmd.driving_pwm = MIN_ALLOWED_BACKWARD;
  }
}

Buggy::Buggy() :
    read_ind(WINDOW_SIZE - 1),
    /*sharp_left(IR_LEFT), sharp_right(IR_RIGHT), sharp_front_left(IR_FRONT_LEFT), sharp_front_right(IR_FRONT_RIGHT), sharp_front(IR_FRONT), */
    steering_pwm(STEERING_NEUTRAL), driving_pwm(STOP), led_state(true) {
}

Buggy::~Buggy() {
  delete adc;
}

void Buggy::setup() {
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_FRONT_LEFT, INPUT);
  pinMode(IR_FRONT, INPUT);
  pinMode(IR_FRONT_RIGHT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  pinMode(BATTERY, INPUT);

  adc = new ADC(); // adc object

  adc->setSamplingSpeed(ADC_VERY_LOW_SPEED); // change the sampling speed
  adc->setConversionSpeed(ADC_VERY_LOW_SPEED); // change the conversion speed
  adc->setResolution(10);
  adc->setAveraging(0);

  /*
   analogReference (DEFAULT);
   analogReadAveraging(16);
   analogReadResolution(10);
   */

  pinMode(TEENSY_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  pinMode(START_BUTTON, INPUT_PULLUP);
  attachInterrupt(START_BUTTON, button_service, RISING);

  steeringservo.attach(STEERING_PWM_PIN);
  drivingservo.attach(DRIVE_PWM_PIN);
}

uint16_t Buggy::battery_voltage() {
  // 3.6 V divided 2.71 times read value
  // ((3600000 / 1024) * analogRead(BATTERY) * 2710) / 1000000
  //return uint16_t((9756 * uint32_t(analogRead(BATTERY))) >> 10);
  return uint16_t((9756 * uint32_t(adc->analogRead(BATTERY))) >> 10);
}

void Buggy::auto_mode_blink(uint32_t time) {
  if (blink_timer.start_or_triggered(time, AUTO_BLINK_INTERVAL_MS, false, true)) {
    digitalWrite(TEENSY_LED, led_state ? HIGH : LOW);
    led_state = !led_state;
  }
}

void Buggy::sense(bc_telemetry_packet_t& telemetry) {
  telemetry.time = millis();

  if (!telemetry.automatic) {
    if (button_pressed) {
      button_pressed = false;

      countdown_timer.start(telemetry.time, START_DELAY_MS, true);
      digitalWrite(TEENSY_LED, HIGH); // inform the user that countdown started
    }

    // ready to race!?
    if (countdown_timer.triggered(telemetry.time)) {
      telemetry.automatic = true;
    }
  }

  //telemetry.ir_left = sharp_left.distance();
  //telemetry.ir_right = sharp_right.distance();
  //telemetry.ir_front_left = sharp_front_left.distance();
  //telemetry.ir_front_right = sharp_front_right.distance();
  //telemetry.ir_front = sharp_front.distance();

  if (ir_check_timer.start_or_triggered(telemetry.time, 1, false, true))
  {
    read_ind = (read_ind + 1) % WINDOW_SIZE;
    reads[0][read_ind] = irLookup[adc->analogRead(IR_LEFT)];
    reads[1][read_ind] = irLookup[adc->analogRead(IR_FRONT_LEFT)];
    reads[2][read_ind] = irLookup[adc->analogRead(IR_FRONT)];
    reads[3][read_ind] = irLookup[adc->analogRead(IR_FRONT_LEFT)];
    reads[4][read_ind] = irLookup[adc->analogRead(IR_RIGHT)];

    uint8_t k, i;
    fixed sum, diff;
    fixed n = WINDOW_SIZE;
    for (k = 0; k < 5; ++k) {
      sum = 0;
      for (i = 0; i < WINDOW_SIZE; ++i) {
        sum = sum + reads[k][i];
      }
      average[k] = sum / n;
      sum = 0;
      for (i = 0; i < WINDOW_SIZE; ++i) {
        diff = reads[k][i] - average[k];
        sum = sum + (diff * diff);
      }
      variance[k] = sum / n;
    }

    telemetry.ir_left = variance[0] < INFINITY_VARIANCE ? average[0] : INFINITY;
    telemetry.ir_front_left = variance[1] < INFINITY_VARIANCE ? average[1] : INFINITY;
    telemetry.ir_front = variance[2] < INFINITY_VARIANCE ? average[2] : INFINITY;
    telemetry.ir_front_right = variance[3] < INFINITY_VARIANCE ? average[3] : INFINITY;
    telemetry.ir_right = variance[4] < INFINITY_VARIANCE ? average[4] : INFINITY;
  }

  //telemetry.ir_left = irLookup[adc->analogRead(IR_LEFT)];
  //telemetry.ir_front_left = irLookup[adc->analogRead(IR_FRONT_LEFT)];
  //telemetry.ir_front = irLookup[adc->analogRead(IR_FRONT)];
  //telemetry.ir_front_right = irLookup[adc->analogRead(IR_FRONT_LEFT)];
  //telemetry.ir_right = irLookup[adc->analogRead(IR_RIGHT)];

  if (battery_check_timer.start_or_triggered(telemetry.time, BATTERY_CHECK_INTERVAL_MS, false, true)) {
    telemetry.battery = battery_voltage();
    digitalWrite(RED_LED, telemetry.battery < BATTERY_LOW_MILLI_VOLTAGE ? HIGH : LOW);
  }
}

void Buggy::act(bc_telemetry_packet_t& telemetry) {
  if (steering_pwm != telemetry.steering_pwm) {
    steering_pwm = telemetry.steering_pwm;
    steeringservo.write(steering_pwm);
  }
  if (driving_pwm != telemetry.driving_pwm) {
    driving_pwm = telemetry.driving_pwm;
    drivingservo.write(driving_pwm);
  }

  if (telemetry.automatic) {
    auto_mode_blink(telemetry.time);
  }
}