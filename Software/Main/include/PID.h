#ifndef PID_H
#define PID_H

namespace micromouse {

class PID {
	
public:
	PID(float kp, float ki, float kd);
	void setSetpoint(float setpoint);
	float update(float input, float dt);
	void reset();

private:
	float _ki;
	float _kp;
	float _kd;
	float _setpoint;
	float _previousError;
	float _integralError;
	
};

}

#endif