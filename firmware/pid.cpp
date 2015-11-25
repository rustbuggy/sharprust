#include "pid.h"

PID::PID() {
  reset();
}

PID::~PID() {
}

void PID::reset() {
  dState = 0.0f;
  iState = 0.0f;
  iMax = 1.0f;
  iMin = -1.0f;
  pGain = 0.1f;
  iGain = 0.001f;
  dGain = 0.0f;
  firstUpdate = true;
}

float PID::update(float error, float position)
{
  if (firstUpdate) {
    firstUpdate = false;
    dState = position;
  }

  float pTerm, dTerm, iTerm;

  // calculate the integral state with appropriate limiting
  iState += error;
  if (iState > iMax) {
    iState = iMax;
  }
  else if (iState < iMin) {
    iState = iMin;
  }

  pTerm = pGain * error; // calculate the proportional term
  iTerm = iGain * iState;  // calculate the integral term
  dTerm = dGain * (position - dState);

  dState = position;

  return pTerm + iTerm - dTerm;
}
