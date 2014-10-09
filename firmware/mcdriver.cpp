#include "mcdriver.h"

static const fx24_8_t VAL_SQRT_1_DIV_2 = 181;

MCDriver::MCDriver()
{
  mc_x = mc_y = 0;
  lastDriveCmd.changeSteering = false;
  lastDriveCmd.steeringPwm = 90;
  lastDriveCmd.changeDriving = false;
  lastDriveCmd.drivingPwm = 190;
}

void MCDriver::clampSteeringAndSpeed()
{
  // Steering
  if (driveCmd.steeringPwm > 140)
  {
    driveCmd.steeringPwm = 140;
  }
  else if (driveCmd.steeringPwm < 40)
  {
    driveCmd.steeringPwm = 40;
  }

  // Speed
  if (driveCmd.drivingPwm > 255)
  {
    driveCmd.drivingPwm = 255;
  }
  else if (driveCmd.drivingPwm < 135)
  {
    driveCmd.drivingPwm = 135;
  }
}

drive_cmd_t& MCDriver::drive(bc_telemetry_packet_t& telemetry)
{
  /*
   // Mass center calculation
   fx24_8_t a1 = FIXED_Mul(VAL_SQRT_1_DIV_2, FIXED_FROM_INT(telemetry.ir_front_right));
   fx24_8_t a2 = FIXED_Mul(VAL_SQRT_1_DIV_2, FIXED_FROM_INT(telemetry.ir_front_left));
   fx24_8_t a3 = FIXED_FROM_INT(telemetry.ir_right);
   fx24_8_t a4 = FIXED_FROM_INT(telemetry.ir_left);
   
   fx24_8_t mc_x = FIXED_Mul(a1 + a3 - a2 - a4, VAL_1_DIV_4);
   fx24_8_t mc_y = FIXED_Mul(a1 + a2, VAL_1_DIV_4);
   */

  int32_t a1 = FIXED_TO_INT(FIXED_Mul(VAL_SQRT_1_DIV_2, FIXED_FROM_INT(telemetry.ir_front_right)));
  int32_t a2 = FIXED_TO_INT(FIXED_Mul(VAL_SQRT_1_DIV_2, FIXED_FROM_INT(telemetry.ir_front_left)));
  int32_t a3 = telemetry.ir_right;
  int32_t a4 = telemetry.ir_left;

  int32_t max_x_neg = a2 < a4 ? a2 : a4;
  int32_t min_x_pos = a1 < a3 ? a1 : a3;
  int32_t min_y_pos = a1 < a2 ? a1 : a2;

  int32_t new_mc_x = (min_x_pos - max_x_neg) / 2;
  int32_t new_mc_y = min_y_pos / 2;
  
  int32_t dist_x = new_mc_x - mc_x;
  int32_t dist_y = new_mc_y - mc_y;
  int32_t distSqr = dist_x * dist_x + dist_y * dist_y;

  // TODO: set suitable steering and driving PWM
  //if (distSqr > 9)
  if (true)
  {
    driveCmd.changeSteering = true;
    driveCmd.steeringPwm = 90 + 5 * new_mc_x;
    driveCmd.changeDriving = true;
    driveCmd.drivingPwm = 184;//190 - 3 * new_mc_y;
  }
  else
  {
    driveCmd.changeSteering = false;
    driveCmd.steeringPwm = lastDriveCmd.steeringPwm;
    driveCmd.changeDriving = false;
    driveCmd.drivingPwm = lastDriveCmd.drivingPwm;
  }
  
  clampSteeringAndSpeed();

  telemetry.mc_x = mc_x = new_mc_x;
  telemetry.mc_y = mc_y = new_mc_y;
  telemetry.changeSteering = driveCmd.changeSteering;
  telemetry.steeringPwm = driveCmd.steeringPwm;
  telemetry.changeDriving = driveCmd.changeDriving;
  telemetry.drivingPwm = driveCmd.drivingPwm;

  lastDriveCmd.changeSteering = driveCmd.changeSteering;
  lastDriveCmd.steeringPwm = driveCmd.steeringPwm;
  lastDriveCmd.changeDriving = driveCmd.changeDriving;
  lastDriveCmd.drivingPwm = driveCmd.drivingPwm;

  return driveCmd;
}

