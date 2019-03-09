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

#define FRONT_WALL_THRESHOLD_CLOSE	2570 // 3000
#define FRONT_WALL_THRESHOLD_FAR	3164 // 3400
#define FRONT_WALL_SPEED_REDUCTION	0.3

#define LEFT_WALL_THRESHOLD			3600
#define RIGHT_WALL_THRESHOLD		3600
#define PROPER_FRONT_WALL_DISTANCE	1600 // 1600
#define PROPER_LEFT_WALL_DISTANCE	2659 // 2800
#define PROPER_RIGHT_WALL_DISTANCE	2725 // 2687

#define WALL_CORRECT_SPEED			50
#define WALL_CORRECT_PERIOD			(int)((DISTANCE_PER_STEP * 1000000)/ WALL_CORRECT_SPEED)
#define WALL_CORRECT_INCREMENT		100

#define UNCALIBRATED_DRIVE_DISTANCE 5

#define CORRECT_DRIFT_INCREMENTS	100
#define CORRECT_DRIFT_ANGLE			0.6
#define CORRECT_DRIFT_DELAY			7500

#define ERROR_TWOWALL_COEFFICIENT	1

#define ERROR_TWOWALL_HEADING_COEFFICIENT   1
#define ERROR_ONEWALL_HEADING_COEFFICIENT   80
#define ERROR_NOWALL_HEADING_COEFFICIENT	80

#define IMU_TOLERANCE				1

#define DRIVE_SPEED			175	// Average drive speed
#define TURN_SPEED			200	// The max speed at which turns are taken
#define TURN_ACCEL_ANGLE	0.3 // The angular distance to accelerate over
#define TURN_ACCEL_DIVS		10	// Number of angular increments to accelerate over
#define ACCEL_MIN_SPEED		50	// The speed to accelerate from / decelerate to
#define DRIVE_DISTANCE		182.5	// The distance of one cell
#define DRIVE_ACCEL_DISTANCE 15 // The distance over which to decelerate
#define DRIVE_ACCEL_DIVS	 10 // The number of divisions to divide a deceleration into

namespace micromouse {
	
class Robot {
	
public:
	int init();
	void reset();
	void enableMotors();
	void disableMotors();
	int pid_drive(void);
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
	int stopSafely();
	
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
