#include "irsharp.h"

#include "Arduino.h"
#include "lookups.h"

IRSharp::IRSharp(int16_t irPin) :
irPin(irPin), dist(40) {
	pinMode(irPin, INPUT);
}

// GP2Y0A21Y
fixed& IRSharp::distance() {
	dist = irLookup[analogRead(irPin)];
	return dist;
}

