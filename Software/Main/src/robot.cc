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
	std::cout << "Initializing robot..." << std::endl;
	
	/** Map PRU memory space **/
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	
	if(fd == -1)
	{
		std::cout << "ERROR: could not open /dev/mem.\n\n" << std::endl;
		return 1;
	}
	
	this->_pru_mem = mmap(0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDR);
	
	if (this->pru_mem == MAP_FAILED)
	{
		std::cout << "ERROR: could not map memory.\n\n" << std::endl;
		return 1;
	}
	close(fd);
	
	unsigned int *pru0_dram = pru_mem + PRU0_DRAM/4 + 0x200/4;   // Points to 0x200 of PRU0 memory
	unsigned int *pru1_dram = pru_mem + PRU1_DRAM/4 + 0x200/4;   // Points to 0x200 of PRU1 memory
	
	/** Initialize drive system **/
	micromouse::GpioDevice* leftMotorDirPin = new GpioDevice(112);
	micromouse::GpioDevice* rightMotorDirPin = new GpioDevice(113);
	micromouse::GpioDevice* enableMotorPin = new GpioDevice(114);
	
	leftMotorDirPin->setDirection(GPIO_OUT);
	rightMotorDirPin->setDirection(GPIO_OUT);
	enableMotorPin->setDirection(GPIO_OUT);
	
	this->_motorSystem = new micromouse::MotorSystem("/dev/rpmsg_pru30",  
													leftMotorDirPin, 
													rightMotorDirPin, 
													enableMotorPin);
	
	std::cout << "Initialized drive system." << std::endl;
	
	/** Initialize BMS **/
	
	/** Initialize sensor subsystem **/
	
	this->_leftFrontFacing = new
		micromouse::ReflSensor("/home/debian/sensors/sensor3",
								ledDriver,
								3);
	
	this->_rightFrontFacing = new
		micromouse::ReflSensor("/home/debian/sensors/sensor5",
								ledDriver,
								5);
								
	this->_frontLeftFacing = new
		micromouse::ReflSensor("/home/debian/sensors/sensor2",
								ledDriver,
								2);
								
	this->_rearLeftFacing = new
		micromouse::ReflSensor("/home/debian/sensors/sensor0",
								ledDriver,
								0);
								
	this->_frontRightFacing = new
		micromouse::ReflSensor("/home/debian/sensors/sensor1",
								ledDriver,
								1);
								
	this->_rearRightFacing = new
		micromouse::ReflSensor("/home/debian/sensors/sensor4",
								ledDriver,
								4);
	
	std::cout << "Initialized reflectance sensors." << std::endl;
	
	/*
	micromouse::I2cDevice* i2c2 = new I2cDevice(2);
	this->_imu = new micromouse::IMU(i2c2, BNO055_ADDRESS_A);
	
	error = this->_imu->Initialize();
	if(error) {
		std::cout << "Error initializing IMU." << std::endl;
		return -1;
	}
	
	std::cout << "Initialized IMU." << std::endl;
	*/
	this->getHeading(&(this->_headingTarget));
	
	std::cout << "Set heading target to " << this->_headingTarget << std::endl;
	
	std::ifstream pidFile;
	pidFile.open(PID_GAINS_PATH);
	pidFile >> this->_driveKp;
	pidFile >> this->_driveKi;
	pidFile >> this->_driveKd;
	pidFile >> this->_driveHeadingCoefficient;
	pidFile.close();
	
	std::cout << "Loaded PID gains: ";
	std::cout << this->_driveKp << " ";
	std::cout << this->_driveKi << " ";
	std::cout << this->_driveKd << std::endl;
	std::cout << "Loaded heading coefficient: " << this->_driveHeadingCoefficient << std::endl;
	
	float distance = 1000;
	while(distance > 50) {
		getFrontDistance(&distance);
	}
	
	this->_led1 = new micromouse::RgbLedDevice(64, 47, 65);
	this->_led1->setRgb(0, 0, 1);
	
	usleep(5000000);
}

void Robot::enableMotors() {
	this->_motorSystem->enable();
}

void Robot::disableMotors() {
	this->_motorSystem->disable();
}

int Robot::getLeftDistance(float* distance) {
	*distance = (this->_frontLeftFacing->getDistance() +
				this->_rearLeftFacing->getDistance())/2.0;
	
	return 0;
}

int Robot::getRightDistance(float* distance) {
	*distance = (this->_frontRightFacing->getDistance() +
				this->_rearRightFacing->getDistance())/2.0;
	
	return 0;
}

int Robot::getFrontDistance(float* distance) {
	*distance = (this->_leftFrontFacing->getDistance() +
				this->_rightFrontFacing->getDistance())/2.0;
	
	return 0;
}

int Robot::frontWallCorrect() {
	float distance;
	this->getFrontDistance(&distance);
	int steps = (int)((distance - PROPER_FRONT_WALL_DISTANCE) / DISTANCE_PER_STEP);
	int period = (int)((DISTANCE_PER_STEP * 1000000)/ WALL_CORRECT_SPEED);
	
	int direction = MOTOR_FORWARD;
	
	//std::cout << "Correcting by " << (distance - PROPER_FRONT_WALL_DISTANCE) << std::endl;
	
	if(steps < 0) {
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

int Robot::pid_drive(float distance, float speed) {
	
	PID pid(this->_driveKp, this->_driveKi, this->_driveKd);
	pid.reset();
	pid.setSetpoint(0);
	
	float heading;
	for(int i = 0; i < DRIVE_DIVISIONS; i++) {
		
		int ret;
		bool wall;
		ret = this->checkWallFront(&wall);
		if(ret) {
			std::cout << "Error checking for wall." << std::endl;
		}
		if(wall) {
			return this->frontWallCorrect();
		}
		
		float temp;
		ret = this->getHeading(&temp);//this->_imu->GetHeading(heading);
		if(ret) {
			std::cout << "Error getting heading." << std::endl;
			//return ret;
		}
		else {
			heading = temp;
		}
		
		float error = this->_driveHeadingCoefficient * 
						DeltaAngle(this->_headingTarget, heading);
		std::cout << error << " ";
		
		// Determine if we have enough walls to calculate error
		bool leftWall = 0, rightWall = 0;
		if(this->checkWallLeft(&leftWall) || 
			this->checkWallRight(&rightWall)) {
			std::cout << "Error checking wall." << std::endl; 
			return -1;
		}
		
		if(leftWall && rightWall) {
			float leftSensorDistance, rightSensorDistance;
			if(this->getLeftDistance(&leftSensorDistance) || 
				this->getRightDistance(&rightSensorDistance)) {
				std::cout << "Error getting distance." << std::endl;
				return -1;
			}
			
			std::cout << (leftSensorDistance - rightSensorDistance) << std::endl;
			error += (leftSensorDistance - rightSensorDistance);
		}
		
		else if(leftWall) {
			std::cout << "Warning: only left wall detected" << std::endl;
			float distance;
			int e;
			if(e = this->getLeftDistance(&distance)) {
				return e;
			}
			
			std::cout << -(PROPER_SIDE_WALL_DISTANCE - distance) << std::endl;
			error += -(PROPER_SIDE_WALL_DISTANCE - distance);
		}
		
		else if(rightWall) {
			std::cout << "Warning: only right wall detected" << std::endl;
			float distance;
			int e;
			if(e = this->getRightDistance(&distance)) {
				return e;
			}
			
			std::cout << -(distance - PROPER_SIDE_WALL_DISTANCE) << std::endl;
			error += -(distance - PROPER_SIDE_WALL_DISTANCE);
		}
		
		else {
			std::cout << "WARNING: NO WALLS DETECTED." << std::endl;
		}
		
		
		float offset = pid.update(error, (distance / DRIVE_DIVISIONS) / speed);
		
		float leftDistance = (distance / DRIVE_DIVISIONS) + offset;
		float rightDistance = (distance / DRIVE_DIVISIONS) - offset;
		
		// First, figure out how much time the entire operation will take using the
		// given velocity
		float time = ((leftDistance + rightDistance) / 2) / speed;
		
		// Now compute the left and right velocities, and accordingly the time per
		// step of the left and right wheels
		float leftVelocity = leftDistance / time;
		float rightVelocity = rightDistance / time;
			
		// (rad/step) / (rad/s) = (s/step)
		// Also calculate required angular velocity
		// v = rw -> w = r/v
		int periodLeft = (int)((DISTANCE_PER_STEP * 1000000)/ 
								leftVelocity);
		int periodRight = (int)((DISTANCE_PER_STEP * 1000000) / 
								rightVelocity);
								
		//std::cout << periodRight << std::endl;
			
		int stepsLeft = leftDistance / DISTANCE_PER_STEP;
		int stepsRight = rightDistance / DISTANCE_PER_STEP;
		
		if(ret = this->_motorSystem->drive(stepsLeft,
									stepsRight,
									periodLeft,
									periodRight,
									MOTOR_FORWARD,
									MOTOR_FORWARD,
									5000) ){
			return ret;
		}
	}

	return 0;
}

int Robot::turn(int amt, float speed) {
	// Adjust the target heading
	this->_headingTarget += amt * 90.0;
	
	while(this->_headingTarget > 360.0) {
		this->_headingTarget -= 360.0;
	}
	
	while(this->_headingTarget < 0) {
		this->_headingTarget += 360.0;
	}
	
	int stepSpeed = (int)((DISTANCE_PER_STEP * 1000000) / speed);
	int ret;
	ret = this->_motorSystem->drive(TURN_STEPS(amt),
									TURN_STEPS(amt),
									stepSpeed,
									stepSpeed,
									((amt > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD),
									((amt > 0) ? MOTOR_BACKWARD : MOTOR_FORWARD),
									5000);
	if(ret) {
		std::cout << "Error turning." << std::endl;
		return ret;
	}
	
	usleep(500000);
	
	float previous, current;
	do {
		ret = getHeading(&previous);
		if(ret) {
			std::cout << "Error reading IMU." << std::endl;
			//return -1;
		}
		usleep(250000);
		ret = getHeading(&current);
		if(ret) {
			std::cout << "Error reading IMU." << std::endl;
			//return -1;
		}
	} while (fabs(previous - current) > IMU_TOLERANCE);
	
	float turnFraction;
	do {
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
		if(ret) {
			std::cout << "Error turning." << std::endl;
			return ret;
		}
	}
	while(fabs(turnFraction) < 0.01);
	
	usleep(500000);
	
	return 0;
}

int Robot::checkWallFront(bool* result) {
	*result = this->_leftFrontFacing->getDistance() <= WALL_THRESHOLD && 
				this->_rightFrontFacing->getDistance() <= WALL_THRESHOLD;
	return 0;
}

int Robot::checkWallRight(bool* result) {
	*result = this->_frontRightFacing->getDistance() <= WALL_THRESHOLD && 
				this->_rearRightFacing->getDistance() <= WALL_THRESHOLD;
	return 0;
}

int Robot::checkWallLeft(bool* result) {
	*result = this->_frontLeftFacing->getDistance() <= WALL_THRESHOLD && 
				this->_rearLeftFacing->getDistance() <= WALL_THRESHOLD;
	return 0;
}

int Robot::getHeading(float* heading) {
	
	std::ifstream imuFile;
	imuFile.open("/home/debian/sensors/imu");
	if(imuFile.fail()) {
		std::cout << "Error opening IMU file." << std::endl;
		return -1;
	}
	
	imuFile >> *heading;
	
	if(imuFile.fail()) {
		std::cout << "Error reading IMU." << std::endl;
		return -1;
	}
	
	imuFile.close();
	return 0;
}

}