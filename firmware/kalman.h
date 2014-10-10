// Idea from: http://pengu.student.utwente.nl/wordpress/?p=116

#ifndef __KALMAN__H
#define __KALMAN__H

#include "types.h"

class SingleKalmanVar
{
private:
  fixed_t Sz; // Q
  fixed_t Sw; // R
  fixed_t P;
  fixed_t x;

public:
  SingleKalmanVar(fixed_t x, fixed_t P, fixed_t Sz, fixed_t Sw);
  void reset(fixed_t x, fixed_t P, fixed_t Sz, fixed_t Sw);
  fixed_t stepKalman(fixed_t measurement);
};

#endif // __KALMAN__H


