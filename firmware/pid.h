#ifndef __PID__H
#define __PID__H


class PID
{
  private:
    float dState;      	// last position input
    float iState;      	// integrator state
    float iMax, iMin;  // maximum and minimum allowable integrator state
    float	iGain;    	// integral gain
    float	pGain;    	// proportional gain
    float	dGain;     	// derivative gain

    bool firstUpdate;

  public:
    PID();
    virtual ~PID();
    void reset();
    float update(float error, float position);
};

#endif // __PID__H
