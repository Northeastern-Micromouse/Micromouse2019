#!/bin/bash
pkill -f imu
echo 0 > /sys/devices/platform/leds/leds/beaglebone\:green\:usr2/brightness
sleep 30
cd /home/debian/PRU/PRU0
make
cd /home/debian/PRU/PRU1
make
cd /home/debian/IMU
gcc imu.c -o imu -lm
./imu &
cd /home/debian/Main/src
make clean && make
./main
