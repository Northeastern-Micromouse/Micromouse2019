/**
 * serial.h
 * Basic linux serial io functions
 */

#ifndef SERIAL_H
#define SERIAL_H

#include <termios.h>

#define DEVICE "/dev/ttyO5"

#define BAUDRATE B115200

// see termios(7)
// to turn on use | and to turn off use &~
#define CCFLAG ~PARENB & ~CSTOPB & ~CSIZE | CS8 & ~CRTSCTS & ~CREAD & ~CLOCAL
#define CIFLAG ~(IXON | IXOFF | IXANY | ICANON | ECHO | ECHOE)

int init_serial(void);
int write_serial(char*, int);

#endif
