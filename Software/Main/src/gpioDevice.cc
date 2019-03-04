#include "gpioDevice.h"

#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>

namespace micromouse {

const static unsigned int GPIO_BUF_SIZE = 50;

GpioDevice::GpioDevice(int pin) {
    this->pin = pin;
	this->unexport(); // Just in case it was previously setup
	
    std::fstream f;
    f.open("/sys/class/gpio/export");
    f << pin;
    f.close();
}

GpioDevice::~GpioDevice() {
    this->unexport();
}

void GpioDevice::unexport()
{
	std::fstream f;
    f.open("/sys/class/gpio/unexport");
    f << this->pin;
    f.close();
}

void GpioDevice::setDirection(int direction) {
    std::fstream f;
    f.open("/sys/class/gpio/gpio" + std::to_string(this->pin) + "/direction");
    if (direction) {
        f << "out";
    } else {
        f << "in";
    }

    this->direction = direction;
    f.close();
}

void GpioDevice::setValue(int value) {
    std::fstream f;
    f.open("/sys/class/gpio/gpio" + std::to_string(this->pin) + "/value");
    f << value;
    f.close();
}

int GpioDevice::getDirection() {
    return this->direction;
}

int GpioDevice::getValue() {
    std::fstream f;
    f.open("/sys/class/gpio/gpio" + std::to_string(this->pin) + "/value");
    int val;
	f >> val;
    f.close();
	return val;
}

}

