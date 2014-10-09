// Idea from: http://pengu.student.utwente.nl/wordpress/?p=116

#ifndef __KALMAN__H
#define __KALMAN__H

#include "types.h"

class SingleKalmanVar
{
private:
  fx24_8_t Sz; // Q
  fx24_8_t Sw; // R
  fx24_8_t P;

public:
  fx24_8_t x;
  SingleKalmanVar(int32_t x, int32_t P, int32_t Sz, int32_t Sw);
  void reset(int32_t x, int32_t P, int32_t Sz, int32_t Sw);
  int32_t stepKalman(int32_t measurement);
};

#endif // __KALMAN__H


