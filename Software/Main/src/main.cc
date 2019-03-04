#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <time.h>

#include "robot.h"
//#include "AlgoRobot.h"
#include "Direction.h"

int main()
{	
	/*
	std::ifstream speedStream;
	speedStream.open("/home/debian/size.txt");
	
	int x, y;
	speedStream >> x;
	speedStream >> y;
	speedStream.close();

	algorithm::Robot winslow = algorithm::Robot(true, x, y, algorithm::Direction::NORTH);
	winslow.Map();
	*/
	
	micromouse::Robot boi = micromouse::Robot();
	
	std::cout << "Initializing robot..." << std::endl;
	boi.init();
	std::cout << "Robot Initialized." << std::endl;
	usleep(1000000);
	/*
	while(1)
	{
		printf(
			"%3.1f\t%3.1f\t%3.1f\t%3.1f\t%3.1f\t%3.1f\n",
			boi.getSensorSystem() -> getLeftDistanceFront(0),
			boi.getSensorSystem() -> getLeftDistanceRear(0),
			boi.getSensorSystem() -> getRightDistanceFront(0),
			boi.getSensorSystem() -> getRightDistanceRear(0),
			boi.getSensorSystem() -> getFrontDistanceLeft(0),
			boi.getSensorSystem() -> getFrontDistanceRight(0)
		);
		usleep(100000);
	}
	*/
	
	while(1)
	{
		printf(
			"%3.1f\t%3.1f\t%3.1f\n",
			boi.getSensorSystem() -> getHeading(),
			boi.getSensorSystem() -> getPitch(),
			boi.getSensorSystem() -> getRoll()
		);
		
		usleep(100000);
	}
	
	/*
	while(1)
	{
		printf(
			"%4.1f\t%4.1f\t%4.1f\n",
			boi.getLeftDistance(),
			boi.getRightDistance(),
			boi.getFrontDistance()
		);
		
		usleep(100000);
	}
	*/
	/*
	while(1)
	{
		if(boi.readButton1())
		{
			std::cout << "Driving..." << std::endl;
			boi.getMotorSystem() -> enable();
			usleep(100000);
			//boi.getMotorSystem() -> drive(400*16,400*16,150,150,true,true,10000);
			//boi.pid_drive(180, 150);
			boi.turn(2,150);
			usleep(100000);
			boi.getMotorSystem() -> disable();
		}
	}
	*/
	/*
	while(!boi.readButton1());
	
	std::cout << "Starting PID drive..." << std::endl;
	boi.getMotorSystem() -> enable();	
	
	while(1)
	{
		for(int i = 0; i < 5; i++)
		{
			boi.pid_drive(180, 150);
		}
		boi.turn(2, 50);
	}
	
	return 0;
	*/
	/*
	while(!boi.readButton1());
	
	std::cout << "Starting THE DONUT" << std::endl;
	boi.getMotorSystem() -> enable();	
	
	while(1)
	{
		for(int i = 0; i < 2; i++)
		{
			boi.pid_drive(180, 150);
		}
		boi.turn(1, 150);
	}
	*/
	return 0;
}
