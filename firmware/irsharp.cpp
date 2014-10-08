#include "Arduino.h"
#include "IRSharp.h"
#include "lookups.h"

IRSharp::IRSharp(int irPin) : dist(40, 10, 1, 5), irPin(irPin)
{
  analogReference(DEFAULT);
}

// GP2Y0A21Y
int IRSharp::distance()
{
  return dist.stepKalman(irLookup[analogRead(irPin)]);
}

