/**
 * arm_sensor_test.c
 * Tests the ARM PRU sensor subsystem.
 */

#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include "pru_sensors.h"

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
	
	pru1_dram[MEM_RDY] = MEM_ARM_RDY_VALUE;
	while(pru1_dram[MEM_RDY] != MEM_RDY_VALUE);
	
	printf("PRU Initialized.\n");

	while(1)
	{
		printf(
			"XL:%d %d %d\t G:%d %d %d\t M:%d %d %d\t A: %d %d %d %d %d %d %d\n",
			pru1_dram[MEM_ACCEL_X],
			pru1_dram[MEM_ACCEL_Y],
			pru1_dram[MEM_ACCEL_Z],
			pru1_dram[MEM_GYRO_X],
			pru1_dram[MEM_GYRO_Y],
			pru1_dram[MEM_GYRO_Z],
			pru1_dram[MEM_MAG_X],
			pru1_dram[MEM_MAG_Y],
			pru1_dram[MEM_MAG_Z],
			pru1_dram[MEM_AIN0],
			pru1_dram[MEM_AIN1],
			pru1_dram[MEM_AIN2],
			pru1_dram[MEM_AIN3],
			pru1_dram[MEM_AIN4],
			pru1_dram[MEM_AIN5],
			pru1_dram[MEM_AIN6]
		);
		
		usleep(100000);
	}
	
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