#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <time.h>

#include "robot.h"
#include "Cell.h"
#include "AlgoRobot.h"
#include "Direction.h"

int main()
{
	micromouse::Robot* boiLL = new micromouse::Robot();

	std::cout << "Initializing..." << std::endl;
	boiLL->init();
	boiLL->enableMotors();
	std::cout << "Initialized." << std::endl;

	std::ofstream ledFile;
	ledFile.open("/sys/devices/platform/leds/leds/beaglebone\:green\:usr1/brightness", std::ios::trunc);
	ledFile << "0";
	ledFile.close();
	ledFile.open("/sys/devices/platform/leds/leds/beaglebone\:green\:usr2/brightness", std::ios::trunc);
	ledFile << "0";
	ledFile.close();
	ledFile.open("/sys/devices/platform/leds/leds/beaglebone\:green\:usr3/brightness", std::ios::trunc);
	ledFile << "0";
	ledFile.close();

	usleep(1000000);

	algorithm::Robot boi = algorithm::Robot(boiLL, true, 16, 16, algorithm::Direction::NORTH);
	boi.Reset(true);

	bool validMaze = false;
	while(!validMaze)
	{
		validMaze = boi.Map();
	}
	
	std::vector<algorithm::Cell*> path = boi.ComputeShortestPath();

	while(1)
	{
		while(!boiLL->readButton1());
		usleep(500000);
		boi.Reset(false);
		boi.Run(path);
	}

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
	/*
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
	*/
	/*
	micromouse::Robot* boi = new micromouse::Robot();

	std::cout << "Initializing..." << std::endl;
	boi->init();
	boi->enableMotors();
	std::cout << "Initialized." << std::endl;

	usleep(1000000);
	
	while(1)
	{
		printf(
			"%4.1f\t%4.1f\t%4.1f\n",
			boi->getLeftDistance(),
			boi->getRightDistance(),
			boi->getFrontDistance()
		);

		usleep(100000);
	}
	*/
	/*
	micromouse::Robot* boi = new micromouse::Robot();

	std::cout << "Initializing..." << std::endl;
	boi->init();
	boi->enableMotors();
	std::cout << "Initialized." << std::endl;

	usleep(1000000);

	while(1)
	{
		if(boi->readButton1())
		{
			std::cout << "Driving..." << std::endl;
			boi->getMotorSystem()->enable();
			usleep(100000);
			//boi.getMotorSystem() -> drive(400*16,400*16,150,150,true,true,10000);
			//boi.pid_drive(180, 150);
			boi->turn(1,TURN_SPEED);
			usleep(100000);
			boi->getMotorSystem()->disable();
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
			boi.pid_drive(180, 200);
		}
		boi.turn(2, 200);
	}

	return 0;
	*/
	/*
	while(1)
	{
		while(!boi.readButton1());

		boi.reset();
		std::cout << "Starting THE DONUT" << std::endl;
		boi.getMotorSystem() -> enable();

		boi.correctDrift();

		while(!boi.readButton2())
		{
			while(!boi.checkWallFrontClose())
			{
				boi.pid_drive();
			}

			usleep(10000);
			boi.turn(1, TURN_SPEED);
		}
	}
	*/
	/*
	while(1)
	{
		boi.printOffHeading();
		usleep(100000);
	}
	return 0;
	*/
}
