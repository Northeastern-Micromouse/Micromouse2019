#include "sensors.h"

namespace micromouse
{

/**
 * init(unsigned int timeout)
 * Initializes the sensor subsystem. Times out after the given number of
 * microseconds.
 * Returns 0 if successful, otherwise returns something non-zero depending
 * on specific error.
 */
int SensorSystem::init(unsigned int* pru1_mem, unsigned int timeout) : _pru1_mem(pru1_mem)
{
	this->_pru1_mem[MEM_RDY] = MEM_ARM_RDY_VALUE;
	
	int counter = 0;
	while(_pru1_mem[MEM_RDY] != MEM_RDY_VALUE)
	{
		if(counter > timeout)
		{
			return 1;
		}
		
		counter++;
		usleep(1);
	}
	
	return 0;
}

float SensorSystem::getHeading()
{
	
}

float SensorSystem::getPitch()
{
	
}

float SensorSystem::getRoll()
{
	
}

float SensorSystem::getLeftDistanceFront();
float SensorSystem::getLeftDistanceRear();
float SensorSystem::getRightDistanceFront();
float SensorSystem::getRightDistanceRear();
float SensorSystem::getFrontDistanceLeft();
float SensorSystem::getFrontDistanceRight();
	
}