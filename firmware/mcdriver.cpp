#include "mcdriver.h"

#include "math.h"

#define STOP 90
#define NORMAL_FORWARD 105
#define NORMAL_BACKWARD 65

MCDriver::MCDriver() {
	last_time = 0;
	stopped_since = 0;

	driveCmd.changeSteering = false;
	driveCmd.steeringPwm = STOP;
	driveCmd.changeDriving = false;
	driveCmd.drivingPwm = STOP;
}

void MCDriver::clampSteeringAndSpeed() {
	// Steering
	if (driveCmd.steeringPwm > 130) {
		driveCmd.steeringPwm = 130;
	}
	else if (driveCmd.steeringPwm < 50) {
		driveCmd.steeringPwm = 50;
	}

	// Speed
	if (driveCmd.drivingPwm > NORMAL_FORWARD) {
		driveCmd.drivingPwm = NORMAL_FORWARD;
	}
	else if (driveCmd.drivingPwm < NORMAL_BACKWARD) {
		driveCmd.drivingPwm = NORMAL_BACKWARD;
	}
}

drive_cmd_t& MCDriver::drive(bc_telemetry_packet_t& telemetry) {
	// Mass center calculation
	fixed_t a1 = FIXED_Mul(VAL_SQRT_1_DIV_2, telemetry.ir_front_right);
	fixed_t a2 = FIXED_Mul(VAL_SQRT_1_DIV_2, telemetry.ir_front_left);
	fixed_t a3 = telemetry.ir_right;
	fixed_t a4 = telemetry.ir_left;
	fixed_t a5 = telemetry.ir_front;

	fixed_t mc_x = FIXED_Mul(VAL_1_DIV_5, a1 + a3 - a2 - a4); // TODO: calculate offset from sensor positioning as well
	fixed_t mc_y = FIXED_Mul(VAL_1_DIV_5, a1 + a2 + a5); // TODO: calculate offset from sensor positioning as well
	fixed_t dist = FIXED_FROM_DOUBLE(sqrt(FIXED_TO_DOUBLE(FIXED_Mul(mc_x, mc_x) + FIXED_Mul(mc_y, mc_y)))); // TODO: get rid of double and sqrt
	fixed_t angle = FIXED_Mul(VAL_RAD_TO_DEG, FIXED_FROM_DOUBLE(atan2(FIXED_TO_DOUBLE(mc_y), FIXED_TO_DOUBLE(mc_x)))); // TODO: get rid of double and ata

	fixed_t minFront = a1;
	if (minFront > a2) {
		minFront = a2;
	}
	if (minFront > a5) {
		minFront = a5;
	}

	driveCmd.changeSteering = true;
	driveCmd.steeringPwm = 90 - (FIXED_TO_INT(angle) - 90);
	driveCmd.changeDriving = true;

	bool maybeStuck = (FIXED_TO_INT(dist) < 10) || (FIXED_TO_INT(minFront) < 10); // replace it with some kind of sensible state machine to get out of stuck state
	if (maybeStuck) {
		if (stopped_since == 0) {
			stopped_since = telemetry.time;
		}
		else if (telemetry.time > stopped_since + 3000) {
			driveCmd.steeringPwm = 90;
			driveCmd.drivingPwm = NORMAL_BACKWARD;
			stopped_since = telemetry.time;
		}

		last_time = telemetry.time;
	}
	else {
		stopped_since = 0;
		driveCmd.drivingPwm = NORMAL_FORWARD;
	}

	clampSteeringAndSpeed();

	// Fill missing telemetry values
	telemetry.mc_x = mc_x;
	telemetry.mc_y = mc_y;
	telemetry.mc_dist = dist;
	telemetry.mc_angle = angle;
	telemetry.change_steering = driveCmd.changeSteering;
	telemetry.steering_pwm = driveCmd.steeringPwm;
	telemetry.change_driving = driveCmd.changeDriving;
	telemetry.driving_pwm = driveCmd.drivingPwm;

	return driveCmd;
}

