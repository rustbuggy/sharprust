#ifndef __DRIVER__H
#define __DRIVER__H

#include "types.h"
#include "communication.h"

typedef struct drive_cmd_t
{
  bool changeSteering;
  int32_t steeringPwm;
  bool changeDriving;
  int32_t drivingPwm;
} drive_cmd_t;

// Abstract base class
class Driver
{
  protected:
  drive_cmd_t driveCmd;
  
  public:
  virtual drive_cmd_t& drive(bc_telemetry_packet_t& telemetry) = 0;
};

#endif // __DRIVER__H
