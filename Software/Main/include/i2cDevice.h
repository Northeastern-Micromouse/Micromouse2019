#ifndef _MMI2C_H_
#define _MMI2C_H_

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#define BUF_SIZE 64

namespace micromouse {

class I2cDevice {

public:
    I2cDevice(int bus);
    ~I2cDevice();

    int openI2C();
    void closeI2C();
    int setAddress(unsigned char address);
    int sendByte(char addr, char reg, char data);
    int readByte(char addr, char reg, char* data);
    int readBytes(char addr, char reg, size_t size, char* data);

private:
    int i2cFile;
	int _bus;
    char read_buf[BUF_SIZE];
    char write_buf[BUF_SIZE];
};

}
#endif
