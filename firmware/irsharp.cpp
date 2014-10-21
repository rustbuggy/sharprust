#include "Arduino.h"
#include "irsharp.h"
#include "lookups.h"

IRSharp::IRSharp(int irPin) :
		dist(FIXED_FROM_INT(40), FIXED_FROM_INT(10), FIXED_FROM_INT(1), FIXED_FROM_INT(5)), irPin(irPin) {
	pinMode(irPin, INPUT);
}

// GP2Y0A21Y
fixed_t IRSharp::distance() {
	return dist.stepKalman(FIXED_FROM_INT(irLookup[analogRead(irPin)]));
}

