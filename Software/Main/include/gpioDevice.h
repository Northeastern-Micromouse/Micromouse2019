#ifndef _MMGPIO_H_
#define _MMGPIO_H_

#include <iostream>
#include <fstream>
#include <string>

namespace micromouse {
	
#define GPIO_OUT 1
#define GPIO_IN	 0

class GpioDevice {

public:
    GpioDevice(int pin);
    ~GpioDevice();
	void unexport(void);

    void setDirection(int direction);
    void setValue(int value);

    int getDirection();
    int getValue();

private:
    int pin;
    int direction;

};

}
#endif
