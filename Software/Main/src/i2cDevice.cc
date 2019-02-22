#include "i2cDevice.h"
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace micromouse {

I2cDevice::I2cDevice(int bus):_bus(bus) {
    this->openI2C();
}

I2cDevice::~I2cDevice() {
    this->closeI2C();
}

int I2cDevice::openI2C() {
    std::cout << "opening i2c-" << this->_bus << std::endl;
    this->i2cFile = open(("/dev/i2c-" + std::to_string(this->_bus)).c_str(), O_RDWR);
    if (this->i2cFile < 0) {
        return i2cFile;
	}
	
	return 0;
}

void I2cDevice::closeI2C() {
    std::cout << "closing i2c-" << _bus << std::endl;
    close(this->i2cFile);
}

int I2cDevice::setAddress(unsigned char address) {
    //std::cout << "setting slave address " << std::hex;
    //std::cout << int(address) << std::endl;
	return ioctl(this->i2cFile, I2C_SLAVE, address);
}

int I2cDevice::sendByte(char addr, char reg, char data) {
    this->setAddress(addr);
    this->write_buf[0] = reg;
    this->write_buf[1] = data;

    //std::cout << "writing " << std::hex << int(data) << std::endl;
    int ret;
	if ((ret = write(this->i2cFile, this->write_buf, 2)) != 2) {
        return ret;
    }
	
	return 0;
}

int I2cDevice::readByte(char addr, char reg, char* data) {
    this->write_buf[0] = reg;
    this->setAddress(addr);
    //std::cout << "write reg value " << std::hex << (int)reg << std::endl;
    int ret;
	if ((ret = write(this->i2cFile, this->write_buf, 1)) != 1) {
        return ret;
    }

    //std::cout << "read byte" << std::endl;
    this->setAddress(addr);
    if ((ret = read(this->i2cFile, this->read_buf, 1)) != 1) {
        return ret;
    }

    *data = this->read_buf[0];
	return 0;
}

int I2cDevice::readBytes(char addr, char reg, size_t size, char* data) {
    this->write_buf[0] = reg;
    this->setAddress(addr);
    //std::cout << "write reg value " << std::hex << (int)reg << std::endl;
    int ret;
	if ((ret = write(this->i2cFile, this->write_buf, 1)) != 1) {
        return ret;
    }

    //std::cout << "read bytes" << std::endl;
    this->setAddress(addr);
    if ((ret = read(this->i2cFile, data, size)) != (int)size) {
        return ret;
    }

	return 0;
} 

}

