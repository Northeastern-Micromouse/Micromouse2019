#ifndef ROBOT_H
#define ROBOT_H

#include <fstream>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include "gpioDevice.h"
#include "ledDriver.h"
#include "motors.h"
#include "rgbLedDevice.h"
#include "PID.h"
#include "reflSensor.h"
#include "imu.h"

// PRU memory address and offsets
#define PRU_ADDR        0x4A300000      // Start of PRU memory Page 184 am335x TRM
#define PRU_LEN         0x80000         // Length of PRU memory
#define PRU0_DRAM       0x00000         // Offset to DRAM
#define PRU1_DRAM       0x02000
#define PRU_SHAREDMEM   0x10000         // Offset to shared memory

// PID Gains for driving
#define PID_GAINS_PATH "/home/debian/pidgains.txt"

// Number of discrete motions a single drive command will be divided into
#define DRIVE_DIVISIONS 15

// Hardware constants
#define MICROSTEPPING			16
#define RADIANS_PER_STEP 		((0.9 / MICROSTEPPING) * M_PI/180.0)
#define WHEEL_RADIUS 			30.0
#define DISTANCE_PER_STEP		(RADIANS_PER_STEP*WHEEL_RADIUS)

#define ROBOT_WIDTH 			110.0
#define TURN_LENGTH				(M_PI/2.0 * ROBOT_WIDTH/2.0)
#define TURN_STEPS(n)			(int)(fabs(n) * TURN_LENGTH / DISTANCE_PER_STEP)

#define WALL_THRESHOLD				80.0 
#define PROPER_FRONT_WALL_DISTANCE	30
#define PROPER_SIDE_WALL_DISTANCE	30
#define WALL_CORRECT_SPEED			100

#define IMU_TOLERANCE				1

#define HEADING_MEASUREMENTS 5

namespace micromouse {
	
class Robot {
	
public:
	int init();
	void enableMotors();
	void disableMotors();
	int pid_drive(float distance, float speed);
	int turn(int amt, float speed);
	int getLeftDistance(float* distance);
	int getRightDistance(float* distance);
	int getFrontDistance(float* distance);
	int checkWallFront(bool* result);
	int checkWallRight(bool* result);
	int checkWallLeft(bool* result);
	int getHeading(float* heading);
	int frontWallCorrect();

private:
	
	unsigned int *_pru_mem;       // Points to start of PRU memory.
	
	micromouse::MotorSystem* _motorSystem;
	micromouse::SensorSystem* _sensorSystem;
	
	float _driveKp;
	float _driveKi;
	float _driveKd;
	
	//micromouse::IMU* _imu;
	float _headingTarget;
	float _driveHeadingCoefficient;
	
};
	
	
}

#endif