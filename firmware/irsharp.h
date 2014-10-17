#ifndef __IRSHARP__H
#define __IRSHARP__H

#include "Arduino.h"
#include "kalman.h"

class IRSharp {
public:
	IRSharp(int irPin);
	fixed_t distance();

private:
	int irPin;
	SingleKalmanVar dist;
};

#endif // __IRSHARP__H

