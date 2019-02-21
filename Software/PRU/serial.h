/**
 * serial.h
 * Basic linux serial io functions
 */

#ifndef SERIAL_H
#define SERIAL_H

#include <termios.h>

#define DEVICE "/dev/ttyUSB0"

#define BAUDRATE B115200

// see termios(7)
// to turn on use | and to turn off use &~
#define CCFLAG ~PARENB & ~CSTOPB & ~CSIZE | CS8 & ~CRTSCTS & ~CREAD & ~CLOCAL
#define CIFLAG ~(IXON | IXOFF | IXANY | ICANON | ECHO | ECHOE | ISOC)

void init_serial(void);
int write_serial(char*);

#endif
