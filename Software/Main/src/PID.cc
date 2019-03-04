#include "PID.h"

namespace micromouse {

PID::PID(float kp, float ki, float kd):
			_kp(kp), _ki(ki), _kd(kd), 
			_previousError(0), _integralError(0) {}

float PID::update(float error, float dt) {
	float output = this->_kp * error + 
					this->_ki * (this->_integralError * dt) +
					this->_kd * (error - this->_previousError)/dt;
					
	this->_previousError = error;
	this->_integralError += error;
	
	return output;
}

void PID::reset() {
	this->_previousError = 0;
	this->_integralError = 0;
}

}
