#include "Arduino.h"
#include "IRSharp.h"
#include "lookups.h"

IRSharp::IRSharp(int irPin) : dist(40, 10, 0.1, 1), irPin(irPin)
{
  analogReference(DEFAULT);
}

// GP2Y0A21Y
int IRSharp::distance()
{
  return dist.stepKalman(irLookup[analogRead(irPin)]);
}

