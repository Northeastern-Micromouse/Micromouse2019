#include "robot.h"

namespace micromouse {

float DeltaAngle(float current, float target)
{
    if (fabs(target - current) > 180)
	{
        return (360 - fabs(target - current))*
                    (current - target > 0 ? 1 : -1);
    }

    return target - current;
}

void Robot::printOffHeading()
{
	std::cout << DeltaAngle(getHeading(), _headingTarget) << std::endl;
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
	std::ifstream configFile;
	configFile.open(CONFIG_PATH);
	configFile >> this->_driveKp;
	configFile >> this->_driveKi;
	configFile >> this->_driveKd;
	configFile >> this->_driveDivisions;
	configFile.close();

	std::cout << "Loaded PID gains: ";
	std::cout << this->_driveKp << " ";
	std::cout << this->_driveKi << " ";
	std::cout << this->_driveKd << std::endl;

	std::cout << "Loaded " << this->_driveDivisions << " drive divisions." << std::endl;

	usleep(1000000);
}

void Robot::reset()
{
	this->_sensorSystem->reset();
	this->_driveCounter = 0;
	correctDrift();
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
	// First, turn so that we're facing the wall head-on
	turn(0, WALL_CORRECT_SPEED);
	usleep(50000);

	while(getFrontDistance() > PROPER_FRONT_WALL_DISTANCE)
	{
		int ret = this->_motorSystem->drive(WALL_CORRECT_INCREMENT,
											WALL_CORRECT_INCREMENT,
											WALL_CORRECT_PERIOD,
											WALL_CORRECT_PERIOD,
											MOTOR_FORWARD,
											MOTOR_FORWARD,
											5000);

		if(ret)
		{
			std::cout << "Error during wall correction." << std::endl;
			return ret;
		}
	}

	return 0;
}

int Robot::pid_drive()
{
	this->_driveCounter++;

	if(this->_driveCounter >= UNCALIBRATED_DRIVE_DISTANCE && checkWallLeft() && checkWallRight())
	{
		usleep(750000);
		correctDrift();
		this->_driveCounter = 0;
	}

	PID pid(this->_driveKp, this->_driveKi, this->_driveKd);
	pid.reset();

	float heading;
	for(int i = 0; i < this->_driveDivisions; i++) {

		if(checkWallFrontClose())
		{
			return this->frontWallCorrect();
		}

		float driveSpeed = checkWallFrontFar() ?
			(ACCEL_MIN_SPEED + (DRIVE_SPEED - ACCEL_MIN_SPEED)*(1 - (getFrontDistance() - FRONT_WALL_THRESHOLD_FAR)/(FRONT_WALL_THRESHOLD_CLOSE - FRONT_WALL_THRESHOLD_FAR)) )
			: DRIVE_SPEED;


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
			//std::cout << "Warning: only left wall detected" << std::endl;
			/*
			error = 0.5*ERROR_ONEWALL_HEADING_COEFFICIENT*DeltaAngle(heading, _headingTarget)
						+ 0.5*(PROPER_LEFT_WALL_DISTANCE - leftDistance);
			*/
			error = PROPER_LEFT_WALL_DISTANCE - leftDistance;
		}

		else if(rightWall)
		{
			//std::cout << "Warning: only right wall detected" << std::endl;
			//std::cout << DeltaAngle(heading, _headingTarget) << std::endl;
			/*
			error = 0.5*ERROR_ONEWALL_HEADING_COEFFICIENT*DeltaAngle(heading, _headingTarget)
						+ 0.5*(rightDistance - PROPER_RIGHT_WALL_DISTANCE);
			*/
			error = rightDistance - PROPER_RIGHT_WALL_DISTANCE;
		}

		else
		{
			//std::cout << "WARNING: NO WALLS DETECTED." << std::endl;
			//std::cout << DeltaAngle(heading, _headingTarget) << std::endl;
			error = ERROR_NOWALL_HEADING_COEFFICIENT*DeltaAngle(heading, _headingTarget);
		}


		float offset = pid.update(error, (DRIVE_DISTANCE / this->_driveDivisions) / driveSpeed);

		float leftDriveDistance = (DRIVE_DISTANCE / this->_driveDivisions) + offset;
		float rightDriveDistance = (DRIVE_DISTANCE / this->_driveDivisions) - offset;

		if(leftDriveDistance <= 0.1)
		{
			leftDriveDistance = 0.1;
		}

		if(rightDriveDistance <= 0.1)
		{
			rightDriveDistance = 0.1;
		}

		//std::cout << "H: " << heading << "\tT: " << _headingTarget << "\tDA: " << DeltaAngle(heading, this->_headingTarget) << "\tE: " << error << "\tO: " << offset << std::endl;

		// First, figure out how much time the entire operation will take using the
		// given velocity
		float time = ((leftDriveDistance + rightDriveDistance) / 2) / driveSpeed;

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

int Robot::correctDrift()
{
	// First, turn about 30 degrees one way
	int ret = this->_motorSystem->drive(TURN_STEPS(CORRECT_DRIFT_ANGLE / 2.0),
									TURN_STEPS(CORRECT_DRIFT_ANGLE / 2.0),
									WALL_CORRECT_PERIOD,
									WALL_CORRECT_PERIOD,
									MOTOR_FORWARD,
									MOTOR_BACKWARD,
									10000);
	if(ret)
	{
		std::cout << "Error turning." << std::endl;
		return ret;
	}

	float minDistance = 999999;
	float minAngle = 0;

	bool useLeft = getLeftDistance() > getRightDistance();

	for(int i = 0; i < CORRECT_DRIFT_INCREMENTS; i++)
	{
		int ret = this->_motorSystem->drive(TURN_STEPS(CORRECT_DRIFT_ANGLE/CORRECT_DRIFT_INCREMENTS),
									TURN_STEPS(CORRECT_DRIFT_ANGLE/CORRECT_DRIFT_INCREMENTS),
									WALL_CORRECT_PERIOD,
									WALL_CORRECT_PERIOD,
									MOTOR_BACKWARD,
									MOTOR_FORWARD,
									10000);
		if(ret)
		{
			std::cout << "Error turning." << std::endl;
			return ret;
		}

		usleep(CORRECT_DRIFT_DELAY);
		float distance = useLeft ? getLeftDistance() : getRightDistance();

		if(distance < minDistance)
		{
			minDistance = distance;
			minAngle = getHeading();
		}
	}

	_headingTarget = minAngle;

	turn(0, WALL_CORRECT_SPEED);

	return 0;
}

int Robot::turn(int amt, float speed) {
	// Wait until gyro is stable
	float previous, current;
	do
	{
		previous = getHeading();
		usleep(50000);
		current = getHeading();
	}
	while(fabs(DeltaAngle(previous, current)) > IMU_TOLERANCE);

	/*
	// If we have three walls surrounding us, calibrate for gyro drift by normalizing against the walls
	if(correctForDrift && checkWallFrontClose() && checkWallLeft() && checkWallRight())
	{
		int retVal = correctDrift();
		if(retVal)
		{
			std::cout << "Error correcting drift." << std::endl;
			return retVal;
		}
	}
	*/

	float offAmount = DeltaAngle(getHeading(), this->_headingTarget) / 90.0;

	float turnAmt = ((float)amt) + offAmount;

	std::cout << "OffAmount: " << offAmount << "\tTurnAmt: " << turnAmt << std::endl;

	this->_headingTarget += amt * 90.0;

	while(this->_headingTarget > 360.0)
	{
		this->_headingTarget -= 360.0;
	}

	while(this->_headingTarget < 0)
	{
		this->_headingTarget += 360.0;
	}

	int period = (int)((DISTANCE_PER_STEP * 1000000) / speed);

	// If amt is 0, we are doing a corrective motion and don't need to implement acceleration
	if(amt == 0)
	{
		int ret;
		ret = this->_motorSystem->drive(TURN_STEPS(turnAmt),
										TURN_STEPS(turnAmt),
										period,
										period,
										((turnAmt > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD),
										((turnAmt > 0) ? MOTOR_BACKWARD : MOTOR_FORWARD),
										10000);
		if(ret)
		{
			std::cout << "Error turning." << std::endl;
			return ret;
		}
	}
	// Otherwise, implement acceleration to prevent slipping
	else
	{
		for(int i = 0; i < TURN_ACCEL_DIVS; i++)
		{
			int steps = TURN_STEPS(TURN_ACCEL_ANGLE / ((float)TURN_ACCEL_DIVS));
			float accelSpeed = ACCEL_MIN_SPEED + (TURN_SPEED - ACCEL_MIN_SPEED)*(((float)i)/((float)TURN_ACCEL_DIVS));
			int accelPeriod = (int)((DISTANCE_PER_STEP * 1000000) / accelSpeed);

			int ret = this->_motorSystem->drive(steps,
											steps,
											accelPeriod,
											accelPeriod,
											((turnAmt > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD),
											((turnAmt > 0) ? MOTOR_BACKWARD : MOTOR_FORWARD),
											10000);
			if(ret)
			{
				std::cout << "Error turning." << std::endl;
				return ret;
			}
		}

		float remainingTurnAmt = (fabs(turnAmt) - 2.0*TURN_ACCEL_ANGLE) * (turnAmt > 0 ? 1 : -1);

		// Turn the rest of the way at typical turn speed
		int ret = this->_motorSystem->drive(TURN_STEPS(remainingTurnAmt),
										TURN_STEPS(remainingTurnAmt),
										period,
										period,
										((remainingTurnAmt > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD),
										((remainingTurnAmt > 0) ? MOTOR_BACKWARD : MOTOR_FORWARD),
										10000);
		if(ret)
		{
			std::cout << "Error turning." << std::endl;
			return ret;
		}

		// Now decelerate out of the turn
		for(int i = 0; i < TURN_ACCEL_DIVS; i++)
		{
			int steps = TURN_STEPS(TURN_ACCEL_ANGLE / ((float)TURN_ACCEL_DIVS));
			float accelSpeed = TURN_SPEED + (ACCEL_MIN_SPEED - TURN_SPEED)*(((float)i)/((float)TURN_ACCEL_DIVS));
			int accelPeriod = (int)((DISTANCE_PER_STEP * 1000000) / accelSpeed);

			int ret = this->_motorSystem->drive(steps,
												steps,
												accelPeriod,
												accelPeriod,
												((turnAmt > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD),
												((turnAmt > 0) ? MOTOR_BACKWARD : MOTOR_FORWARD),
												10000);
			if(ret)
			{
				std::cout << "Error turning." << std::endl;
				return ret;
			}
		}
	}

	usleep(50000);

	// Correct again just in case
	for (int i = 0; i < 2; i++) {
		offAmount = DeltaAngle(getHeading(), this->_headingTarget) / 90.0;
		std::cout << "Corrective OffAmount " << offAmount << std::endl;
	}
	if(fabs(offAmount) > 0.03)
	{
		int ret = this->_motorSystem->drive(TURN_STEPS(offAmount),
											TURN_STEPS(offAmount),
											WALL_CORRECT_PERIOD,
											WALL_CORRECT_PERIOD,
											((offAmount > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD),
											((offAmount > 0) ? MOTOR_BACKWARD : MOTOR_FORWARD),
											10000);
		if(ret)
		{
			std::cout << "Error turning." << std::endl;
			return ret;
		}
	}

	return 0;
}

bool Robot::checkWallFrontClose()
{
	return this->getFrontDistance() <= FRONT_WALL_THRESHOLD_CLOSE;
}

bool Robot::checkWallFrontFar()
{
	return this->getFrontDistance() <= FRONT_WALL_THRESHOLD_FAR;
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
	return 360 - (this->_sensorSystem->getHeading());
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
