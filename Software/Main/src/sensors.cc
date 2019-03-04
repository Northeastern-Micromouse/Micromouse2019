#include "sensors.h"

namespace micromouse
{

SensorSystem::SensorSystem(volatile unsigned int* pru1_mem) : 
							_pru_mem(pru1_mem) {}

/**
 * int init()
 * Initializes the sensor subsystem. 
 * Returns 0 if successful, otherwise returns something non-zero depending
 * on specific error.
 */
int SensorSystem::init()
{
	// We assume the IMU tracker initialized the PRU already
	
	// Now initialize shared memory with the IMU process
	// ftok to generate unique key 
    key_t key = ftok("imufile",65); 
  
    // shmget returns an identifier in shmid 
    int shmid = shmget(key,1024,0666|IPC_CREAT); 
  
    // shmat to attach to shared memory 
    this->_imu_memory = (float*) shmat(shmid,(void*)0,0); 
	
	return 0;
}

float SensorSystem::getHeading()
{
	return this->_imu_memory[2];
}

float SensorSystem::getPitch()
{
	return this->_imu_memory[1];
}

float SensorSystem::getRoll()
{
	return this->_imu_memory[0];
}

float SensorSystem::getLeftDistanceFront(float angle)
{	
	return (float)(this->_pru_mem[MEM_SENSORS_LEFT_DIST_FRONT]);
}

float SensorSystem::getLeftDistanceRear(float angle)
{	
	return (float)(this->_pru_mem[MEM_SENSORS_LEFT_DIST_REAR]);
}

float SensorSystem::getRightDistanceFront(float angle)
{	
	return (float)(this->_pru_mem[MEM_SENSORS_RIGHT_DIST_FRONT]);
}

float SensorSystem::getRightDistanceRear(float angle)
{
	return (float)(this->_pru_mem[MEM_SENSORS_RIGHT_DIST_REAR]);
}

float SensorSystem::getFrontDistanceLeft(float angle)
{
	return (float)(this->_pru_mem[MEM_SENSORS_FRONT_DIST_LEFT]);
}

float SensorSystem::getFrontDistanceRight(float angle)
{
	return (float)(this->_pru_mem[MEM_SENSORS_FRONT_DIST_RIGHT]);
}
	
}