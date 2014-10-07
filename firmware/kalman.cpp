// Idea from: http://pengu.student.utwente.nl/wordpress/?p=116

#include "kalman.h"

// Single variable Kalman filter
SingleKalmanVar::SingleKalmanVar(fx24_8_t x, fx24_8_t P, fx24_8_t Sz, fx24_8_t Sw)
{
  reset(x, P, Sz, Sw);
}

void SingleKalmanVar::reset(fx24_8_t x, fx24_8_t P, fx24_8_t Sz, fx24_8_t Sw)
{
  this->x = x; // variable estimate
  this->P = P; // error (variance) estimate
  this->Sz = Sz; // Q process variance
  this->Sw = Sw; // R measurement variance
}

int32_t SingleKalmanVar::stepKalman(int32_t measurement)
{
  fx24_8_t P_temp, x_temp, K;

  // predict
  x_temp = x;
  P_temp = P + Sw;

  // update
  K = FIXED_Div(P_temp, P_temp + Sz);
  x = x_temp + FIXED_Mul(K, FIXED_FROM_INT(measurement) - x_temp);
  P = FIXED_Mul(FIXED_ONE - K, P_temp);
  
  return FIXED_TO_INT(x);
}

