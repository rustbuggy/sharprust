#include "stdio.h"
#include "mcdriver.h"
#include "communication.h"

MCDriver driver;
bc_telemetry_packet_t telemetry;

int main() {
    telemetry.time           = 0;
    telemetry.ir_left        = 0;
    telemetry.ir_right       = 0;
    telemetry.ir_front_left  = 0;
    telemetry.ir_front_right = 0;
    telemetry.ir_front       = 5;

    for ( int val = 0; val<95; val++ ) {
        telemetry.time += 10;
        telemetry.ir_left  =     val;
        telemetry.ir_right = 94-val;
        drive_cmd_t& dc = driver.drive(telemetry);

        printf("l %2d r %2d steer %3d drive %3d\n", 
            telemetry.ir_left,
            telemetry.ir_right,
            dc.steeringPwm,
            dc.drivingPwm 
        );
    }


}