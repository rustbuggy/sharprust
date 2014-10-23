#ifndef __TYPES__H
#define __TYPES__H

#include <stdint.h>

#include <math.h>

typedef int32_t int_t;
typedef uint32_t uint_t;

typedef int32_t fixed_t;
typedef int64_t fixed_upscale_t;

#define FIXED_BITS          32
#define FIXED_WBITS         16
#define FIXED_FBITS         16
#define FIXED_ONE           (fixed_t(fixed_t(1) << FIXED_FBITS))

#define FIXED_K             (fixed_t(1) << (FIXED_FBITS - 1))

#define FIXED_FROM_INT(a)   (fixed_t(fixed_t(a) << FIXED_FBITS))
#define FIXED_TO_INT(a)     (int_t(a >> FIXED_FBITS))

#define FIXED_ONE_FRECP      (1.0f / float(FIXED_ONE))
#define FIXED_FROM_FLOAT(a) (fixed_t(float(a) * float(FIXED_ONE)))
#define FIXED_TO_FLOAT(a)   (float(a) * FIXED_ONE_FRECP)

#define FIXED_ONE_DRECP      (1.0 / double(FIXED_ONE))
#define FIXED_FROM_DOUBLE(a) (fixed_t(double(a) * double(FIXED_ONE)))
#define FIXED_TO_DOUBLE(a)   (double(a) * FIXED_ONE_DRECP)

#define VAL_RAD_TO_DEG FIXED_FROM_DOUBLE(57.2957795)

// Values for fixed point 16.16
#define VAL_SQRT_1_DIV_2 46340
#define VAL_1_DIV_2 32768
#define VAL_1_DIV_4 16384
#define VAL_3_DIV_4 49152
#define VAL_1_DIV_3 21845
#define VAL_1_DIV_5 13107
#define VAL_1_DIV_10 6553
#define VAL_9_DIV_10 58982

inline fixed_t FIXED_Mul(fixed_t a, fixed_t b) {
	fixed_upscale_t temp = fixed_upscale_t(a) * fixed_upscale_t(b); // result type is operand's type
	temp += FIXED_K; // Rounding; mid values are rounded up

	return fixed_t(temp >> FIXED_FBITS); // Correct by dividing by base
}

inline fixed_t FIXED_Div(fixed_t a, fixed_t b) {
	// pre-multiply by the base
	fixed_upscale_t temp = fixed_upscale_t(fixed_upscale_t(a) << FIXED_FBITS);
	// So the result will be rounded ; mid values are rounded up.
	temp += b / 2;
	return fixed_t(temp / b);
}

struct __attribute__((packed)) point_t {
	fixed_t x, y;

	point_t() :
			x(0), y(0) {
	}

	point_t(fixed_t x, fixed_t y) :
			x(x), y(y) {
	}

	fixed_t get_distance() {
		// TODO: get rid of double and sqrt
		return FIXED_FROM_DOUBLE(sqrt(FIXED_TO_DOUBLE(FIXED_Mul(x, x) + FIXED_Mul(y, y))));
	}

	fixed_t get_deg_angle() {
		// TODO: get rid of double and atan2
		return FIXED_Mul(VAL_RAD_TO_DEG, FIXED_FROM_DOUBLE(atan2(FIXED_TO_DOUBLE(y), FIXED_TO_DOUBLE(x))));
	}
};

#endif // __TYPES__H

