#include "robot.h"

namespace micromouse {
	
float DeltaAngle(float current, float target)
{
    if (fabs(target - current) > 180)
	{
        return (360 - fabs(target - current))*
                    (target - current > 0 ? 1 : -1);
    }
	
    return target - current;
}
	
int Robot::init()
{	
	/** Map PRU memory space **/
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	
	if(fd == -1)
	{
		std::cout << "ERROR: could not open /dev/mem.\n\n" << std::endl;
		return 1;
	}
	
	this->_pru_mem = (unsigned int*)mmap(0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDR);
	
	if (this->_pru_mem == MAP_FAILED)
	{
		std::cout << "ERROR: could not map PRU memory.\n\n" << std::endl;
		return 1;
	}
	close(fd);
	
	volatile unsigned int *pru0_dram = this->_pru_mem + PRU0_DRAM/4 + 0x200/4;   // Points to 0x200 of PRU0 memory
	volatile unsigned int *pru1_dram = this->_pru_mem + PRU1_DRAM/4 + 0x200/4;   // Points to 0x200 of PRU1 memory
	
	/** Initialize drive system **/
	micromouse::GpioDevice* leftMotorDirPin = new GpioDevice(111);
	micromouse::GpioDevice* rightMotorDirPin = new GpioDevice(113);
	micromouse::GpioDevice* enableMotorPin = new GpioDevice(115);
	
	leftMotorDirPin->setDirection(GPIO_OUT);
	rightMotorDirPin->setDirection(GPIO_OUT);
	enableMotorPin->setDirection(GPIO_OUT);
	
	this->_motorSystem = new micromouse::MotorSystem(pru0_dram,  
													leftMotorDirPin, 
													rightMotorDirPin, 
													enableMotorPin);
													
	this->_motorSystem->disable();
	
	if(this->_motorSystem->init(10000))
	{
		std::cout << "ERROR: Drive system initialization timed out." << std::endl;
		return 1;
	}
	
	std::cout << "Initialized drive system." << std::endl;
	
	/** Initialize BMS **/
	
	/** Initialize sensor subsystem **/
	this->_sensorSystem = new micromouse::SensorSystem(pru1_dram);
	if(this->_sensorSystem->init())
	{
		std::cout << "ERROR: Sensor system initialization error." << std::endl;
	}
	
	this->_headingTarget = this->getHeading();
	std::cout << "Set heading target to " << this->_headingTarget << std::endl;
	
	/** Initialize button reading **/
	this->_button1 = new GpioDevice(49);
	this->_button2 = new GpioDevice(117);
	this->_button1->setDirection(GPIO_IN);
	this->_button2->setDirection(GPIO_IN);
	
	/** Load PID gains **/
	std::ifstream pidFile;
	pidFile.open(PID_GAINS_PATH);
	pidFile >> this->_driveKp;
	pidFile >> this->_driveKi;
	pidFile >> this->_driveKd;
	pidFile.close();
	
	std::cout << "Loaded PID gains: ";
	std::cout << this->_driveKp << " ";
	std::cout << this->_driveKi << " ";
	std::cout << this->_driveKd << std::endl;

	usleep(1000000);
}

void Robot::enableMotors() {
	this->_motorSystem->enable();
}

void Robot::disableMotors() {
	this->_motorSystem->disable();
}

float Robot::getLeftDistance() {
	return (this->_sensorSystem->getLeftDistanceFront(getHeading()) + 
			this->_sensorSystem->getLeftDistanceRear(getHeading())) / 2.0;
}

float Robot::getRightDistance() {
	return (this->_sensorSystem->getRightDistanceFront(getHeading()) + 
			this->_sensorSystem->getRightDistanceRear(getHeading())) / 2.0;
}

float Robot::getFrontDistance() {
	return (this->_sensorSystem->getFrontDistanceRight(getHeading()) + 
			this->_sensorSystem->getFrontDistanceLeft(getHeading())) / 2.0;
}

int Robot::frontWallCorrect()
{
	float distance = getFrontDistance();
	int steps = (int)((distance - PROPER_FRONT_WALL_DISTANCE) / DISTANCE_PER_STEP);
	int period = (int)((DISTANCE_PER_STEP * 1000000)/ WALL_CORRECT_SPEED);
	
	int direction = MOTOR_FORWARD;
	
	//std::cout << "Correcting by " << (distance - PROPER_FRONT_WALL_DISTANCE) << std::endl;
	
	if(steps < 0)
	{
		steps = -steps;
		direction = MOTOR_BACKWARD;
	}
	
	return this->_motorSystem->drive(steps,
								steps,
								period,
								period,
								direction,
								direction,
								5000);
								
	usleep(500000);
}

int Robot::pid_drive(float distance, float speed)
{
	
	PID pid(this->_driveKp, this->_driveKi, this->_driveKd);
	pid.reset();
	
	float heading;
	for(int i = 0; i < DRIVE_DIVISIONS; i++) {
		
		if(checkWallFront())
		{
			//std::cout << "Correcting..." << std::endl;
			//return this->frontWallCorrect();
		}
		
		float heading = this->getHeading();
		
		// Determine what information we can gather
		bool leftWall = checkWallLeft();
		bool rightWall = checkWallRight();
		
		float error = 0;
		float leftDistance = getLeftDistance();
		float rightDistance = getRightDistance();
		
		if(leftWall && rightWall)
		{
			error = rightDistance - leftDistance;
		}
		
		else if(leftWall)
		{
			std::cout << "Warning: only left wall detected" << std::endl;
			error = PROPER_LEFT_WALL_DISTANCE - leftDistance;
		}
		
		else if(rightWall)
		{
			std::cout << "Warning: only right wall detected" << std::endl;
			error = PROPER_RIGHT_WALL_DISTANCE - rightDistance;
		}
		
		else {
			std::cout << "WARNING: NO WALLS DETECTED." << std::endl;
			error = DeltaAngle(heading, _headingTarget);
		}
		
		
		float offset = pid.update(error, (distance / DRIVE_DIVISIONS) / speed);
		
		float leftDriveDistance = (distance / DRIVE_DIVISIONS) + offset;
		float rightDriveDistance = (distance / DRIVE_DIVISIONS) - offset;
		
		//std::cout << "E: " << error << "\tL:" << leftDistance << "\tR:" << rightDistance << std::endl; 
				
		// First, figure out how much time the entire operation will take using the
		// given velocity
		float time = ((leftDriveDistance + rightDriveDistance) / 2) / speed;
		
		// Now compute the left and right velocities, and accordingly the time per
		// step of the left and right wheels
		float leftVelocity = leftDriveDistance / time;
		float rightVelocity = rightDriveDistance / time;
			
		// (rad/step) / (rad/s) = (s/step)
		// Also calculate required angular velocity
		// v = rw -> w = r/v
		int periodLeft = (int)((DISTANCE_PER_STEP * 1000000)/ 
								leftVelocity);
		int periodRight = (int)((DISTANCE_PER_STEP * 1000000) / 
								rightVelocity);
								
		//std::cout << periodRight << std::endl;
			
		int stepsLeft = leftDriveDistance / DISTANCE_PER_STEP;
		int stepsRight = rightDriveDistance / DISTANCE_PER_STEP;
				
		int ret;
		if(ret = this->_motorSystem->drive(stepsLeft,
									stepsRight,
									periodLeft,
									periodRight,
									MOTOR_FORWARD,
									MOTOR_FORWARD,
									5000) )
		{
			return ret;
		}
	}

	return 0;
}

int Robot::turn(int amt, float speed) {
	// Adjust the target heading
	/*
	this->_headingTarget += amt * 90.0;
	
	while(this->_headingTarget > 360.0)
	{
		this->_headingTarget -= 360.0;
	}
	
	while(this->_headingTarget < 0)
	{
		this->_headingTarget += 360.0;
	}
	*/
	int stepSpeed = (int)((DISTANCE_PER_STEP * 1000000) / speed);
	int ret;
	ret = this->_motorSystem->drive(TURN_STEPS(amt),
									TURN_STEPS(amt),
									stepSpeed,
									stepSpeed,
									((amt > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD),
									((amt > 0) ? MOTOR_BACKWARD : MOTOR_FORWARD),
									10000);
	if(ret)
	{
		std::cout << "Error turning." << std::endl;
		return ret;
	}
	
	usleep(500000);
	/*
	float previous, current;
	do
	{
		previous = getHeading();
		usleep(250000);
		current = getHeading();
	} 
	while (fabs(previous - current) > IMU_TOLERANCE);
	
	float turnFraction;
	do
	{
		turnFraction = DeltaAngle(this->_headingTarget, current) / 90.0;
		std::cout << "Heading " << current << std::endl;
		std::cout << "Target " << _headingTarget << std::endl;
		std::cout << "Turning " << turnFraction * 90.0 << std::endl;
		ret = this->_motorSystem->drive(TURN_STEPS(turnFraction),
										TURN_STEPS(turnFraction),
										stepSpeed * 4,
										stepSpeed * 4,
										((amt > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD),
										((amt > 0) ? MOTOR_BACKWARD : MOTOR_FORWARD),
										5000);
		if(ret)
		{
			std::cout << "Error turning." << std::endl;
			return ret;
		}
	}
	while(fabs(turnFraction) < 0.01);
	
	usleep(500000);
	*/
	return 0;
}

bool Robot::checkWallFront()
{
	float angle = fabs(DeltaAngle(_headingTarget, getHeading()));
	return this->_sensorSystem->getFrontDistanceLeft(angle) <= FRONT_WALL_THRESHOLD && 
				this->_sensorSystem->getFrontDistanceRight(angle) <= FRONT_WALL_THRESHOLD;
}

bool Robot::checkWallRight()
{
	float angle = fabs(DeltaAngle(_headingTarget, getHeading()));
	return this->_sensorSystem->getRightDistanceFront(angle) <= RIGHT_WALL_THRESHOLD && 
				this->_sensorSystem->getRightDistanceRear(angle) <= RIGHT_WALL_THRESHOLD;
}

bool Robot::checkWallLeft()
{
	float angle = fabs(DeltaAngle(_headingTarget, getHeading()));
	return this->_sensorSystem->getLeftDistanceFront(angle) <= LEFT_WALL_THRESHOLD && 
				this->_sensorSystem->getLeftDistanceRear(angle) <= LEFT_WALL_THRESHOLD;
}

float Robot::getHeading()
{
	return this->_sensorSystem->getHeading();
}

MotorSystem* Robot::getMotorSystem()
{
	return this->_motorSystem;
}

SensorSystem* Robot::getSensorSystem()
{
	return this->_sensorSystem;
}

bool Robot::readButton1()
{
	return this->_button1->getValue() == 0;
}

bool Robot::readButton2()
{
	return this->_button2->getValue() == 0;
}

}