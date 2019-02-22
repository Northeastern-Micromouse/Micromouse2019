#include "PID.h"

namespace micromouse {

PID::PID(float kp, float ki, float kd):
			_kp(kp), _ki(ki), _kd(kd), 
			_setpoint(0), _previousError(0), _integralError(0) {}

void PID::setSetpoint(float setpoint) {
	this->_setpoint = setpoint;
}

float PID::update(float input, float dt) {
	float error = this->_setpoint - input;
	
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
