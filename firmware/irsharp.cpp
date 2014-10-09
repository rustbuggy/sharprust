#include "Arduino.h"
#include "IRSharp.h"
#include "lookups.h"

IRSharp::IRSharp(int irPin) : 
dist(40, 10, 1, 10), irPin(irPin), ewma(0)
{
  pinMode(irPin, INPUT);
}

// GP2Y0A21Y
int IRSharp::distance()
{
  int32_t lastRead = FIXED_TO_INT(dist.x);
  uint8_t newRead = irLookup[analogRead(irPin)];
  int32_t diff = newRead - lastRead;
  fx24_8_t diffSqr = FIXED_FROM_INT(diff * diff);
  ewma = FIXED_Mul(VAL_1_DIV_10, diffSqr) + FIXED_Mul(VAL_9_DIV_10, ewma);
  if (FIXED_TO_INT(ewma) > 10)
  {
    dist.stepKalman(newRead);
    return 100;
  }
  else
  {
    return dist.stepKalman(newRead);
  }
}


