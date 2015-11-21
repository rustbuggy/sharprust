#ifndef PINS_H_
#define PINS_H_

#include <Arduino.h>

//#define TEENSY_LED          13 // (LED)
#define START_BUTTON        18 // A4
#define RED_LED             15 // A1

//#define SPI_SCLK            14 // A0, SCK
#define SPI_SCLK            13 // (LED)
#define SPI_MOSI            11 // DOUT
#define SPI_MISO            12 // DIN
#define SPI_CS              10 // CS

#define BATTERY             16 // A2

#define IR_LEFT             23 // A9
#define IR_FRONT_LEFT       22 // A8
#define IR_FRONT            17 // A3
#define IR_FRONT_RIGHT      21 // A7
#define IR_RIGHT            20 // A6

#define STEERING_PWM_PIN    4
#define DRIVE_PWM_PIN       3

#endif // PINS_H_
