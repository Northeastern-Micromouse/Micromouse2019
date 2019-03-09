/**
 * ahrs_calibaration.c
 * see https://raw.githubusercontent.com/adafruit/Adafruit_AHRS/master/examples/ahrs_calibration/ahrs_calibration.ino
 */

#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include "../PRU/PRU1/pru_sensors.h"

// This program can be used to output raw sensor data in a format that
// can be understoof by MotionCal from PJRC. Download the application
// from http://www.pjrc.com/store/prop_shield.html and make note of the
// magentic offsets after rotating the sensors sufficiently.
//
// You should end up with 3 offsets for X/Y/Z, which are displayed
// in the top-right corner of the application.

#define PRU_ADDR        0x4A300000      // Start of PRU memory Page 184 am335x TRM
#define PRU_LEN         0x80000         // Length of PRU memory
#define PRU0_DRAM       0x00000         // Offset to DRAM
#define PRU1_DRAM       0x02000
#define PRU_SHAREDMEM   0x10000         // Offset to shared memory

int main(int argc, char *argv[])
{
	unsigned int *pru_mem;       // Points to start of PRU memory.
	int fd;

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	
	if(fd == -1)
	{
		printf ("ERROR: could not open /dev/mem.\n\n");
		return 1;
	}
	
	pru_mem = mmap(0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDR);
	
	if (pru_mem == MAP_FAILED)
	{
		printf ("ERROR: could not map memory.\n\n");
		return 1;
	}
	close(fd);
	
	printf("Successfully mapped /dev/mem.\n");

	unsigned int *pru1_dram = pru_mem + PRU1_DRAM/4 + 0x200/4;   // Points to 0x200 of PRU0 memory
	
	printf("Sending initialization signal to PRU...\n");
	
	pru1_dram[MEM_SENSORS_RDY] = MEM_SENSORS_ARM_RDY_VALUE;
	while(pru1_dram[MEM_SENSORS_RDY] != MEM_SENSORS_RDY_VALUE);
	
	printf("PRU Initialized.\n");

	FILE *f = fopen("data.txt", "w+");

	printf("File initialized.\n");
	
	int i = 0;
	for(; i < 10000; i++)
	{
		char text[100];
		
		int len = sprintf( text,
			//"Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
			"%d,%d,%d\r\n",
			//pru1_dram[MEM_ACCEL_X],
			//pru1_dram[MEM_ACCEL_Y],
			//pru1_dram[MEM_ACCEL_Z],
			//pru1_dram[MEM_GYRO_X],
			//pru1_dram[MEM_GYRO_Y],
			//pru1_dram[MEM_GYRO_Z],
			pru1_dram[MEM_SENSORS_MAG_X],
			pru1_dram[MEM_SENSORS_MAG_Y],
			pru1_dram[MEM_SENSORS_MAG_Z]
			//pru1_dram[MEM_MAG_X_C],
			//pru1_dram[MEM_MAG_Y_C],
			//pru1_dram[MEM_MAG_Z_C]
		);
		
		fputs(text, f);
		
		usleep(10000);
	}
	
	fclose(f);
	
	if(munmap(pru_mem, PRU_LEN))
	{
		printf("munmap failed\n");
	} 
	else
	{
		printf("munmap succeeded\n");
	}

	return 0;
}
