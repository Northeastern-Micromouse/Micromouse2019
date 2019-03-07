#ifndef ROBOT_H
#define ROBOT_H

#include <fstream>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include "gpioDevice.h"
#include "motors.h"
#include "PID.h"
#include "sensors.h"

// PRU memory address and offsets
#define PRU_ADDR        0x4A300000      // Start of PRU memory Page 184 am335x TRM
#define PRU_LEN         0x80000         // Length of PRU memory
#define PRU0_DRAM       0x00000         // Offset to DRAM
#define PRU1_DRAM       0x02000
#define PRU_SHAREDMEM   0x10000         // Offset to shared memory

// PID Gains for driving
#define CONFIG_PATH "/home/debian/config.txt"

// Hardware constants
#define MICROSTEPPING			8
#define RADIANS_PER_STEP 		((0.9 / MICROSTEPPING) * M_PI/180.0)
#define WHEEL_RADIUS 			30.0
#define DISTANCE_PER_STEP		(RADIANS_PER_STEP*WHEEL_RADIUS)

#define ROBOT_WIDTH 			78.0
#define TURN_LENGTH				(M_PI/2.0 * ROBOT_WIDTH/2.0)
#define TURN_STEPS(n)			(int)(fabs(n) * TURN_LENGTH / DISTANCE_PER_STEP)

#define FRONT_WALL_THRESHOLD_CLOSE	3170
#define FRONT_WALL_THRESHOLD_FAR	3400
#define FRONT_WALL_SPEED_REDUCTION	0.3

#define LEFT_WALL_THRESHOLD			3600
#define RIGHT_WALL_THRESHOLD		3600
#define PROPER_FRONT_WALL_DISTANCE	1600
#define PROPER_LEFT_WALL_DISTANCE	2800
#define PROPER_RIGHT_WALL_DISTANCE	2687

#define WALL_CORRECT_SPEED			75
#define WALL_CORRECT_PERIOD			(int)((DISTANCE_PER_STEP * 1000000)/ WALL_CORRECT_SPEED)
#define WALL_CORRECT_INCREMENT		100

#define UNCALIBRATED_DRIVE_DISTANCE	10

#define ERROR_TWOWALL_HEADING_COEFFICIENT   1
#define ERROR_ONEWALL_HEADING_COEFFICIENT   80
#define ERROR_NOWALL_HEADING_COEFFICIENT	80

#define IMU_TOLERANCE				1

#define DRIVE_SPEED			200
#define TURN_SPEED			200

namespace micromouse {
	
class Robot {
	
public:
	int init();
	void reset();
	void enableMotors();
	void disableMotors();
	int pid_drive(float distance, float speed);
	int correctDrift(void);
	int turn(int amt, float speed);
	float getLeftDistance(void);
	float getRightDistance(void);
	float getFrontDistance(void);
	bool checkWallFrontFar(void);
	bool checkWallFrontClose(void);
	bool checkWallRight(void);
	bool checkWallLeft(void);
	float getHeading(void);
	int frontWallCorrect(void);
	bool readButton1(void);
	bool readButton2(void);
	
	SensorSystem* getSensorSystem(void);
	MotorSystem* getMotorSystem(void);
	
	void printOffHeading(void);

private:
	
	volatile unsigned int *_pru_mem;       // Points to start of PRU memory.
	
	micromouse::MotorSystem* _motorSystem;
	micromouse::SensorSystem* _sensorSystem;
	
	micromouse::GpioDevice* _button1;
	micromouse::GpioDevice* _button2;
	
	// Tuning parameters
	float _driveKp;
	float _driveKi;
	float _driveKd;
	int _driveDivisions;

	float _headingTarget;
	int _driveCounter = 0;
};
	
	
}

#endif