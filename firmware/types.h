#ifndef __TYPES__H
#define __TYPES__H

#include <stdint.h>
#include <math.h>
#include "fixed.h"

const fixed VAL_RAD_TO_DEG(57.295779513082320876798154814105);

class __attribute__((packed)) point_t {
public:
	fixed x, y;

	point_t() :
			x(0), y(0) {
	}

	point_t(const fixed& x, const fixed& y) :
			x(x), y(y) {
	}

	fixed get_distance() {
		// TODO: get rid of double and sqrt
		return fixed(sqrt(double(x * x + y * y)));
	}

	fixed get_deg_angle() {
		// TODO: get rid of double and atan2
		return fixed(atan2(double(y), double(x))) * VAL_RAD_TO_DEG;
	}
};

#endif // __TYPES__H

