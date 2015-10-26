#ifndef __IRSHARP__H
#define __IRSHARP__H

#include "types.h"

class IRSharp {
public:
  IRSharp(int16_t irPin);
  fixed& distance();

private:
  int16_t irPin;
  fixed dist;
};

#endif // __IRSHARP__H

