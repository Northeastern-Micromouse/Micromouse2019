#!/bin/bash
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