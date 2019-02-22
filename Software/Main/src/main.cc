#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <fstream>

#include "robot.h"
#include "AlgoRobot.h"
#include "Direction.h"

int main()
{	
	std::ifstream speedStream;
	speedStream.open("/home/debian/size.txt");
	
	int x, y;
	speedStream >> x;
	speedStream >> y;
	speedStream.close();

	algorithm::Robot winslow = algorithm::Robot(true, x, y, algorithm::Direction::NORTH);
	winslow.Map();

	return 0;
}
