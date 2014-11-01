#include "fixed.h"

fx_t fixed::temp = 0;
int fixed::loop = 0;
fx_ext_t fixed::temp_ext = 0;

const uint8_t fixed::FIXED_BITS = sizeof(fx_t) * 8;
const uint8_t fixed::FIXED_FBITS = sizeof(fx_t) / 2 * 8;
const fx_t fixed::FIXED_ONE = fx_t(fx_t(1) << FIXED_FBITS);
const fx_ext_t fixed::FIXED_K = fx_ext_t(fx_t(1) << (FIXED_FBITS - 1));
const double fixed::FIXED_ONE_D = double(FIXED_ONE);
const double fixed::FIXED_ONE_D_RECP = 1.0 / FIXED_ONE_D;


fixed operator+(const int a, const fixed& b) {
	return fixed(fixed::fixed_from_int(a) + b.value, true);
}

fixed operator+(const double a, const fixed& b) {
	return fixed(fixed::fixed_from_double(a) + b.value, true);
}

fixed operator-(const int a, const fixed& b) {
	return fixed(fixed::fixed_from_int(a) - b.value, true);
}

fixed operator-(const double a, const fixed& b) {
	return fixed(fixed::fixed_from_double(a) - b.value, true);
}
fixed operator*(const int a, const fixed& b) {
	return fixed(fixed::fixed_multiply(fixed::fixed_from_int(a), b.value), true);
}

fixed operator*(const double a, const fixed& b) {
	return fixed(fixed::fixed_multiply(fixed::fixed_from_double(a), b.value), true);
}

fixed operator/(const int a, const fixed& b) {
	return fixed(fixed::fixed_divide(fixed::fixed_from_int(a), b.value), true);
}

fixed operator/(const double a, const fixed& b) {
	return fixed(fixed::fixed_divide(fixed::fixed_from_double(a), b.value), true);
}
