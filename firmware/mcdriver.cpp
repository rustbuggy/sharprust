#include "mcdriver.h"

#include "math.h"

MCDriver::MCDriver()
{
  mc_x = mc_y = 0;
  lastDriveCmd.changeSteering = false;
  lastDriveCmd.steeringPwm = 90;
  lastDriveCmd.changeDriving = false;
  lastDriveCmd.drivingPwm = 92;
  last_time = 0;
}

void MCDriver::clampSteeringAndSpeed()
{
  // Steering
  if (driveCmd.steeringPwm > 140)
  {
    driveCmd.steeringPwm = 140;
  }
  else if (driveCmd.steeringPwm < 50)
  {
    driveCmd.steeringPwm = 50;
  }

  // Speed
  if (driveCmd.drivingPwm > 110)
  {
    driveCmd.drivingPwm = 110;
  }
  else if (driveCmd.drivingPwm < 80)
  {
    driveCmd.drivingPwm = 80;
  }
}

drive_cmd_t& MCDriver::drive(bc_telemetry_packet_t& telemetry)
{
  // Mass center calculation
  fixed_t a1 = FIXED_Mul(VAL_SQRT_1_DIV_2, telemetry.ir_front_right);
  fixed_t a2 = FIXED_Mul(VAL_SQRT_1_DIV_2, telemetry.ir_front_left);
  fixed_t a3 = telemetry.ir_right;
  fixed_t a4 = telemetry.ir_left;
  
  fixed_t a1a3 = a1 + a3;
  //fixed_t a2a4 = a2 + a4;
  //bool x_positive = a1a3 > a2a4 ? true : false;

  fixed_t new_mc_x, new_mc_y, angle, dist;
  /*
  if (x_positive)
  {
    new_mc_x = FIXED_Mul(VAL_1_DIV_4, a1a3 - a2a4);
  }
  else
  {
    new_mc_x = FIXED_Mul(VAL_1_DIV_4, a2a4 - a1a3);
  }
  */
  new_mc_x = FIXED_Mul(VAL_1_DIV_4, a1 + a3 - a2 - a4);
  new_mc_y = FIXED_Mul(VAL_1_DIV_4, a1 + a2);

  dist = FIXED_FROM_DOUBLE(sqrt(FIXED_TO_DOUBLE(FIXED_Mul(new_mc_x, new_mc_x) + FIXED_Mul(new_mc_y, new_mc_y))));
  angle = FIXED_Mul(VAL_RAD_TO_DEG, FIXED_FROM_DOUBLE(atan2(FIXED_TO_DOUBLE(new_mc_y), FIXED_TO_DOUBLE(new_mc_x)))); // angle
  /*
  if (!x_positive)
  {
    fixed_t diffWithNinety = FIXED_FROM_INT(90) - angle;
    angle += diffWithNinety + diffWithNinety;
  }
  */

  driveCmd.changeSteering = true;
  driveCmd.steeringPwm = 90 - (FIXED_TO_INT(angle) - 90);
  driveCmd.changeDriving = true;
  if (telemetry.time < last_time + 20)
  {
    driveCmd.drivingPwm = 90;  //90 - FIXED_TO_INT(dist);
  }
  else 
  {
    driveCmd.drivingPwm = 92;
    if (telemetry.time > last_time + 25)
    {
      last_time = telemetry.time;
    }
  }

  clampSteeringAndSpeed();

  telemetry.mc_x = mc_x = new_mc_x;
  telemetry.mc_y = mc_y = new_mc_y;
  telemetry.mc_dist = dist;
  telemetry.mc_angle = angle;
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


