/**
 * serial.c
 * Basic linux serial io functions
 */

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include "serial.h"

static int fd;

int init_serial(void)
{
  // open serial device rw and disable tty.
  fd = open(DEVICE, O_RDWR | O_NOCTTY);
  if (fd == -1) { return 0; } // couldn't open DEVICE

  // populate Settings struct from device on fd.
  struct termios Settings;
  tcgetattr(fd, &Settings);

  // set baud rate (in and out).
  cfsetispeed(&Settings, BAUDRATE);
  cfsetospeed(&Settings, BAUDRATE);

  // set bitwise flags.
  Settings.c_cflag &= CCFLAG;
  Settings.c_iflag &= CIFLAG;

  // setup fd to use configured Settings struct.
  tcsetattr(fd, TCSANOW, &Settings);

  return 1; // success
}

int write_serial(char* buffer, int len)
{
  return write(fd, buffer, len);
}
