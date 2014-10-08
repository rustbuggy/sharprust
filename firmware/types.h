#ifndef __TYPES__H
#define __TYPES__H

typedef unsigned char uint8_t;
typedef long int int32_t;

typedef int32_t fx24_8_t;

#define FIXED_BITS        32
#define FIXED_WBITS       24
#define FIXED_FBITS       8
#define FIXED_TO_INT(a)   ( (int32_t)(a >> FIXED_FBITS))
#define FIXED_FROM_INT(a) ((fx24_8_t)(a << FIXED_FBITS))
#define FIXED_ONE         ((fx24_8_t)(1 << FIXED_FBITS))

inline fx24_8_t FIXED_Mul(fx24_8_t x, fx24_8_t y)
{
  return (x * y) >> FIXED_FBITS;
}

inline fx24_8_t FIXED_Div(fx24_8_t x, fx24_8_t y)
{
  return (x << FIXED_FBITS) / y;
}

#endif // __TYPES__H

