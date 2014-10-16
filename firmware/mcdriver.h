#ifndef __MCDRIVER__H
#define __MCDRIVER__H

#include "driver.h"

// Drive towards mass center of estimated IR reflection points
class MCDriver : public Driver {
	protected:
		uint32_t last_time, stopped_since;

		void clampSteeringAndSpeed();

	public:
		MCDriver();
		drive_cmd_t& drive(bc_telemetry_packet_t& telemetry);
};

#endif // __MCDRIVER__H


