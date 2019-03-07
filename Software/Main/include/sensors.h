#ifndef SENSOR_SYSTEM_H
#define SENSOR_SYSTEM_H

#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ipc.h> 
#include <sys/shm.h> 
#include <math.h>
#include "../../PRU/PRU1/pru_sensors.h"

// Subsystem constants
#define MEM_SENSORS_RIGHT_DIST_FRONT	MEM_SENSORS_AIN1
#define MEM_SENSORS_RIGHT_DIST_REAR		MEM_SENSORS_AIN2
#define MEM_SENSORS_LEFT_DIST_FRONT		MEM_SENSORS_AIN4
#define MEM_SENSORS_LEFT_DIST_REAR		MEM_SENSORS_AIN5
#define MEM_SENSORS_FRONT_DIST_LEFT		MEM_SENSORS_AIN0
#define MEM_SENSORS_FRONT_DIST_RIGHT	MEM_SENSORS_AIN3

#define PRU_ADDR        0x4A300000      // Start of PRU memory Page 184 am335x TRM
#define PRU_LEN         0x80000         // Length of PRU memory
#define PRU0_DRAM       0x00000         // Offset to DRAM
#define PRU1_DRAM       0x02000
#define PRU_SHAREDMEM   0x10000         // Offset to shared memory

// The sensor fit is nested; each fit parameter has its own fit curve
// based on angle
// Each element of the array corresponds to an analog input
// Fit parameter A has fit parameters A,B,C,D
#define INDEX_LEFT_DIST_FRONT	0
#define INDEX_LEFT_DIST_REAR	1
#define INDEX_RIGHT_DIST_FRONT	2
#define INDEX_RIGHT_DIST_REAR	3
#define INDEX_FRONT_DIST_LEFT	4
#define INDEX_FRONT_DIST_RIGHT	5

const float REFL_A_A[] = {0,0,0,0,0,0,0};
const float REFL_A_B[] = {0,0,0,0,0,0,0};
const float REFL_A_C[] = {0,0,0,0,0,0,0};
const float REFL_A_D[] = {0,0,0,0,0,0,0};

// Parameter B has fit parameters p1, p2, p3
const float REFL_B_P1[] = {0,0,0,0,0,0,0};
const float REFL_B_P2[] = {0,0,0,0,0,0,0};
const float REFL_B_P3[] = {0,0,0,0,0,0,0};

// Parameter C has fit parameters p1, p2, p3
const float REFL_C_P1[] = {0,0,0,0,0,0,0};
const float REFL_C_P2[] = {0,0,0,0,0,0,0};
const float REFL_C_P3[] = {0,0,0,0,0,0,0};

// Parameter D has fit parameters p1, p2, p3
const float REFL_D_P1[] = {0,0,0,0,0,0,0};
const float REFL_D_P2[] = {0,0,0,0,0,0,0};
const float REFL_D_P3[] = {0,0,0,0,0,0,0};

namespace micromouse 
{
	
class SensorSystem
{
	
public:
	SensorSystem(volatile unsigned int*);
	int init(void);
	void reset(void);
	void zeroIMU(void);
	float getHeading(void);
	float getPitch(void);
	float getRoll(void);
	float getLeftDistanceFront(float);
	float getLeftDistanceRear(float);
	float getRightDistanceFront(float);
	float getRightDistanceRear(float);
	float getFrontDistanceLeft(float);
	float getFrontDistanceRight(float);
	
private:
	volatile unsigned int *_pru_mem;       // Points to start of PRU1 DRAM.
	float* _imu_memory;
};
	
}

#endif