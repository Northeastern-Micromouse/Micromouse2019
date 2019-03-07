#!/bin/bash
cd /home/debian/PRU/PRU0
make
cd /home/debian/PRU/PRU1
make
cd /home/debian/IMU
./imu &
cd /home/debian/Main/src
./main
