// Idea from: http://pengu.student.utwente.nl/wordpress/?p=116

#include "kalman.h"

// Single variable Kalman filter
SingleKalmanVar::SingleKalmanVar(fixed_t x, fixed_t P, fixed_t Sz, fixed_t Sw)
{
  reset(x, P, Sz, Sw);
}

void SingleKalmanVar::reset(fixed_t x, fixed_t P, fixed_t Sz, fixed_t Sw)
{
  this->x = x; // variable estimate
  this->P = P; // error (variance) estimate
  this->Sz = Sz; // Q process variance
  this->Sw = Sw; // R measurement variance
}

fixed_t SingleKalmanVar::stepKalman(fixed_t measurement)
{
  fixed_t P_temp, x_temp, K;

  // predict
  x_temp = x;
  P_temp = P + Sw;

  // update
  K = FIXED_Div(P_temp, P_temp + Sz);
  x = x_temp + FIXED_Mul(K, measurement - x_temp);
  P = FIXED_Mul(FIXED_ONE - K, P_temp);
  
  return x;
}

