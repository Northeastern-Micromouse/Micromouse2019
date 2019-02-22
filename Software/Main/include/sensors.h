#ifndef SENSOR_SYSTEM_H
#define SENSOR_SYSTEM_H

#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "../../PRU/pru_sensors.h"

// Subsystem constants
#define MEM_LEFT_DIST_FRONT		MEM_AIN0
#define MEM_LEFT_DIST_REAR		MEM_AIN1
#define MEM_RIGHT_DIST_FRONT	MEM_AIN2
#define MEM_RIGHT_DIST_REAR		MEM_AIN3
#define MEM_FRONT_DIST_LEFT		MEM_AIN4
#define MEM_FRONT_DIST_RIGHT	MEM_AIN5

#define PRU_ADDR        0x4A300000      // Start of PRU memory Page 184 am335x TRM
#define PRU_LEN         0x80000         // Length of PRU memory
#define PRU0_DRAM       0x00000         // Offset to DRAM
#define PRU1_DRAM       0x02000
#define PRU_SHAREDMEM   0x10000         // Offset to shared memory

namespace micromouse 
{
	
class SensorSystem
{
	
public:
	int init(unsigned int*, unsigned int);
	float getHeading(void);
	float getPitch(void);
	float getRoll(void);
	float getLeftDistanceFront(void);
	float getLeftDistanceRear(void);
	float getRightDistanceFront(void);
	float getRightDistanceRear(void);
	float getFrontDistanceLeft(void);
	float getFrontDistanceRight(void);
	
private:
	unsigned int *_pru1_mem;       // Points to start of PRU1 DRAM.
	
}
	
}

#endif