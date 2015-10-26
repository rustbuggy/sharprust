#ifndef COMMUNICATOR_H_
#define COMMUNICATOR_H_

#include <Arduino.h>
#include "hdlc.h"
#include "packets.h"
#include "timer.h"

// comment out for USB serial
#define USE_XBEE_FOR_TELEMETRY

#ifdef USE_XBEE_FOR_TELEMETRY
#define SERIALDEV Serial3
#else
#define SERIALDEV Serial
#endif

class Communicator {
private:
  drive_cmd_t drive_cmd;

  uint8_t rx_buffer[255];
  uint8_t rx_len;
  uint8_t tx_buffer[255];
  uint8_t tx_len;
  HDLC hdlc;

  timer send_telemetry_timer;

public:
  Communicator();
  virtual ~Communicator();

  void setup();
  void send_telemetry(bc_telemetry_packet_t& telemetry);
  drive_cmd_t& read_command(bc_telemetry_packet_t& telemetry);
};

#endif // COMMUNICATOR_H_
