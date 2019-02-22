#include <fcntl.h>
#include <unistd.h>
#include "motors.h"

namespace micromouse {
	
MotorSystem::MotorSystem(std::string pruFile,
							GpioDevice* leftDirPin,
							GpioDevice* rightDirPin,
							GpioDevice* enablePin):
							_pruFile(pruFile),
							_leftDirPin(leftDirPin),
							_rightDirPin(rightDirPin),
							_enablePin(enablePin) {}
			
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
							
	unsigned char data[] = {(unsigned char)(stepsLeft & 0xFF), 
							(unsigned char)((stepsLeft & 0x0000FF00) >> 8),
							(unsigned char)((stepsLeft & 0x00FF0000) >> 16),
							(unsigned char)((stepsLeft & 0xFF000000) >> 24),
							(unsigned char)((periodLeft & 0x000000FF)),
							(unsigned char)((periodLeft & 0x0000FF00) >> 8),
							(unsigned char)((periodLeft & 0x00FF0000) >> 16),
							(unsigned char)((periodLeft & 0xFF000000) >> 24),
							(unsigned char)(stepsRight & 0xFF), 
							(unsigned char)((stepsRight & 0x0000FF00) >> 8),
							(unsigned char)((stepsRight & 0x00FF0000) >> 16),
							(unsigned char)((stepsRight & 0xFF000000) >> 24),
							(unsigned char)((periodRight & 0x000000FF)),
							(unsigned char)((periodRight & 0x0000FF00) >> 8),
							(unsigned char)((periodRight & 0x00FF0000) >> 16),
							(unsigned char)((periodRight & 0xFF000000) >> 24)	};
							
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
	
	int pruFileFd = open(_pruFile.c_str(), O_RDWR);
    if (pruFileFd < 0) {
		std::cout << "Error opening PRU0 RPMSG file." << std::endl;
        return pruFileFd;
	}
	
	int error;
	if((error = write(pruFileFd, data, 16)) != 16) {
		close(pruFileFd);
		std::cout << "Error writing PRU0 file, returned " << error << std::endl;
		return error;
	}
	
	while(timeout--) {
		char j;
		if(read(pruFileFd, &j, 1) == 1) {
			close(pruFileFd);
			return 0;
		}
		
		usleep(1000);
	}
	
	close(pruFileFd);
	return 1;
}

}