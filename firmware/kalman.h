// From: http://pengu.student.utwente.nl/wordpress/?p=116

#ifndef __KALMAN__H
#define __KALMAN__H

#define KALMAN_TYPE float

class SingleKalmanVar
{
private:
  KALMAN_TYPE x;
  KALMAN_TYPE P;
  KALMAN_TYPE Sz; // Q
  KALMAN_TYPE Sw; // R

public:
  SingleKalmanVar(KALMAN_TYPE x, KALMAN_TYPE P, KALMAN_TYPE Sz, KALMAN_TYPE Sw);
  KALMAN_TYPE stepKalman(KALMAN_TYPE measurement);
};

#endif // __KALMAN__H

