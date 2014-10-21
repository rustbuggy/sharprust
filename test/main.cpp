#include "stdio.h"
#include "math.h"
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
    telemetry.ir_front       = 94;

    for ( int val = 0; val<95; val++ ) {
        telemetry.time += 10;
        telemetry.ir_left           =     val;
        telemetry.ir_right          =  94-val;
        telemetry.ir_front_left     = sqrt(2)*telemetry.ir_left;
        telemetry.ir_front_right    = sqrt(2)*telemetry.ir_right;
        drive_cmd_t& dc = driver.drive(telemetry);

        printf("l %2d r %2d fl %3d fr %3d steer %3d drive %3d\n", 
            telemetry.ir_left,
            telemetry.ir_right,
            telemetry.ir_front_left,
            telemetry.ir_front_right,
            dc.steeringPwm-90,
            dc.drivingPwm-90 
        );
    }


}