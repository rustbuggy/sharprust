#include "kalman.h"

// Single variable Kalman filter
SingleKalmanVar::SingleKalmanVar(KALMAN_TYPE x, KALMAN_TYPE P, KALMAN_TYPE Sz, KALMAN_TYPE Sw)
{
  this->x = x; // variable estimate
  this->P = P; // error (variance) estimate
  this->Sz = Sz; // Q process variance
  this->Sw = Sw; // R measurement variance
}

KALMAN_TYPE SingleKalmanVar::stepKalman(KALMAN_TYPE measurement)
{
  KALMAN_TYPE P_temp, K, x_temp;

  // predict
  x_temp = x;
  P_temp = P + Sw;

  // update
  K = (1 / (P_temp + Sz)) * P_temp;
  x = x_temp + K * (measurement - x_temp);
  P = (1 - K) * P_temp;
  
  return x;
}

