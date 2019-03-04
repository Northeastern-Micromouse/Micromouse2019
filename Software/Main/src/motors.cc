#include <fcntl.h>
#include <unistd.h>
#include "motors.h"

namespace micromouse {
	
MotorSystem::MotorSystem(volatile unsigned int* pru_mem,
							GpioDevice* leftDirPin,
							GpioDevice* rightDirPin,
							GpioDevice* enablePin):
							_pru_mem(pru_mem),
							_leftDirPin(leftDirPin),
							_rightDirPin(rightDirPin),
							_enablePin(enablePin) {
								
								
							}
							
int MotorSystem::init(unsigned int timeout) {
	this->_pru_mem[MEM_MOTORS_RDY] = MEM_MOTORS_ARM_RDY_VALUE;
	while(this->_pru_mem[MEM_MOTORS_RDY] != MEM_MOTORS_RDY_VALUE)
	{
		if(timeout == 0)
		{
			return 1;
		}
		
		usleep(1000);
		timeout--;
	}
	
	return 0;
}
			
void MotorSystem::enable() {
	this->_enablePin->setValue(0);
}

void MotorSystem::disable() {
	this->_enablePin->setValue(1);
}
			
int MotorSystem::drive(unsigned int stepsLeft,
						unsigned int stepsRight,
						unsigned int periodLeft,
						unsigned int periodRight,
						bool directionLeft,
						bool directionRight,
						unsigned int timeout) {
							
	if(directionLeft) {
		//std::cout << "left forward" << !MOTOR_INVERT_LEFT << std::endl;
		this->_leftDirPin->setValue(!MOTOR_INVERT_LEFT);
	}
	else {
		//std::cout << "left backward" << MOTOR_INVERT_LEFT << std::endl;
		this->_leftDirPin->setValue(MOTOR_INVERT_LEFT);
	}
	
	if(directionRight) {
		//std::cout << "right forward" << !MOTOR_INVERT_RIGHT << std::endl;
		this->_rightDirPin->setValue(!MOTOR_INVERT_RIGHT);
	}
	else {
		//std::cout << "right backward" << MOTOR_INVERT_RIGHT << std::endl;
		this->_rightDirPin->setValue(MOTOR_INVERT_RIGHT);
	}
	
	this->_pru_mem[MEM_MOTORS_PERIOD_LEFT] = periodLeft;
	this->_pru_mem[MEM_MOTORS_PERIOD_RIGHT] = periodRight;
	this->_pru_mem[MEM_MOTORS_STEPS_LEFT] = stepsLeft;
	this->_pru_mem[MEM_MOTORS_STEPS_RIGHT] = stepsRight;
	
	while(this->_pru_mem[MEM_MOTORS_STEPS_LEFT] > 0 || this->_pru_mem[MEM_MOTORS_STEPS_RIGHT] > 0) {
		timeout--;
		if(timeout == 0)
		{
			return 1;
		}
		
		usleep(1000);
	}
	
	return 0;
}

}