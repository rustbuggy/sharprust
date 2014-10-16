#include "mcdriver.h"

#include "math.h"

MCDriver::MCDriver() {
	last_time = 0;
	stopped_since = 0;
}

void MCDriver::clampSteeringAndSpeed() {
	// Steering
	if (driveCmd.steeringPwm > 140) {
		driveCmd.steeringPwm = 140;
	}
	else if (driveCmd.steeringPwm < 50) {
		driveCmd.steeringPwm = 50;
	}

	// Speed
	if (driveCmd.drivingPwm > 110) {
		driveCmd.drivingPwm = 110;
	}
	else if (driveCmd.drivingPwm < 80) {
		driveCmd.drivingPwm = 80;
	}
}

drive_cmd_t& MCDriver::drive(bc_telemetry_packet_t& telemetry) {
	// Mass center calculation
	fixed_t a1 = FIXED_Mul(VAL_SQRT_1_DIV_2, telemetry.ir_front_right);
	fixed_t a2 = FIXED_Mul(VAL_SQRT_1_DIV_2, telemetry.ir_front_left);
	fixed_t a3 = telemetry.ir_right;
	fixed_t a4 = telemetry.ir_left;

	fixed_t mc_x = FIXED_Mul(VAL_1_DIV_4, a1 + a3 - a2 - a4);
	fixed_t mc_y = FIXED_Mul(VAL_1_DIV_4, a1 + a2);
	fixed_t dist = FIXED_FROM_DOUBLE(sqrt(FIXED_TO_DOUBLE(FIXED_Mul(mc_x, mc_x) + FIXED_Mul(mc_y, mc_y)))); // TODO: get rid of double and sqrt
	fixed_t angle = FIXED_Mul(VAL_RAD_TO_DEG, FIXED_FROM_DOUBLE(atan2(FIXED_TO_DOUBLE(mc_y), FIXED_TO_DOUBLE(mc_x)))); // TODO: get rid of double and ata

	fixed_t minFront = a1 < a2 ? a1 : a2;

	driveCmd.changeSteering = true;
	driveCmd.steeringPwm = 90 - (FIXED_TO_INT(angle) - 90);
	driveCmd.changeDriving = true;
	bool maybeStuck = FIXED_TO_INT(dist) < 10 || minFront < 15;
	if (!maybeStuck) {
		stopped_since = 0;
		if (telemetry.time < last_time + 20) {
			driveCmd.drivingPwm = 90; //92 - FIXED_TO_INT(dist);
		}
		else {
			driveCmd.drivingPwm = 92;
			if (telemetry.time > last_time + 25) {
				last_time = telemetry.time;
			}
		}
	}
	else {
		if (stopped_since == 0) {
			stopped_since = telemetry.time;
		}
		else if (telemetry.time > stopped_since + 3000) {
			driveCmd.steeringPwm = 90;
			driveCmd.drivingPwm = 100;
		}
		else {
			driveCmd.drivingPwm = 91;
		}

		last_time = telemetry.time;
	}

	clampSteeringAndSpeed();

	// Fill missing telemetry values
	telemetry.mc_x = mc_x;
	telemetry.mc_y = mc_y;
	telemetry.mc_dist = dist;
	telemetry.mc_angle = angle;
	telemetry.changeSteering = driveCmd.changeSteering;
	telemetry.steeringPwm = driveCmd.steeringPwm;
	telemetry.changeDriving = driveCmd.changeDriving;
	telemetry.drivingPwm = driveCmd.drivingPwm;

	return driveCmd;
}


