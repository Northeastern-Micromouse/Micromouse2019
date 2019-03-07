/**
 * imu.c
 * Reads the IMU at a constant interval, filters the data, and writes the results into shared memory
 */

#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ipc.h> 
#include <sys/shm.h> 
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include "../PRU/PRU1/pru_sensors.h"

#define PRU_ADDR        0x4A300000      // Start of PRU memory Page 184 am335x TRM
#define PRU_LEN         0x80000         // Length of PRU memory
#define PRU0_DRAM       0x00000         // Offset to DRAM
#define PRU1_DRAM       0x02000
#define PRU_SHAREDMEM_SENSORS   0x10000         // Offset to shared memory

// System constants
#define deltat 0.023f // sampling period in seconds (shown as 1 ms)
#define gyroMeasError 3.14159265358979 * (2.75f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift 3.14159265358979 * (0.2f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift // compute zeta

// Global system variables
float a_x, a_y, a_z; // accelerometer measurements
float w_x, w_y, w_z; // gyroscope measurements in rad/s
float m_x, m_y, m_z; // magnetometer measurements
float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0; // estimated orientation quaternion elements with initial conditions
float b_x = 1, b_z = 0; // reference direction of flux in earth frame
float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error

// Sensor offsets
int32_t accel_x_bias = 0;
int32_t accel_y_bias = 0;
int32_t accel_z_bias = 0;
int32_t gyro_x_bias = 0;
int32_t gyro_y_bias = 0;
int32_t gyro_z_bias = 0;

void filterUpdate(float,float,float,float,float,float,float,float,float);
void sleep_until(struct timespec *ts, int delay);

int main() 
{ 
    // ftok to generate unique key 
    key_t key = ftok("imufile",65); 
  
    // shmget returns an identifier in shmid 
    int shmid = shmget(key,1024,0666|IPC_CREAT); 
  
    // shmat to attach to shared memory 
    float *data = (float*) shmat(shmid,(void*)0,0); 
  
	// Map PRU memory
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
	
	int timeout = 20000;
	pru1_dram[MEM_SENSORS_RDY] = MEM_SENSORS_ARM_RDY_VALUE;
	while(pru1_dram[MEM_SENSORS_RDY] != MEM_SENSORS_RDY_VALUE)
	{
		if(timeout == 0)
		{
			printf("ERROR: PRU not responding.\n");
			return 1;
		}
		
		timeout--;
		
		usleep(1000);
	}
	
	printf("PRU Initialized.\n");
	
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);

	// Lock memory to ensure no swapping is done.
	if(mlockall(MCL_FUTURE|MCL_CURRENT))
	{
		fprintf(stderr,"WARNING: Failed to lock memory\n");
	}

	// Set our thread to real time priority
	struct sched_param sp;
	sp.sched_priority = 30;
	if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp))
	{
		fprintf(stderr,"WARNING: Failed to set thread to real-time priority\n");
	}
	
	// Collect some samples for biasing
	
	for(int i = 0; i < 128; i++)
	{
		accel_x_bias += (int32_t)pru1_dram[MEM_SENSORS_ACCEL_X];
		accel_y_bias += (int32_t)pru1_dram[MEM_SENSORS_ACCEL_Y];
		accel_z_bias += ((int32_t)pru1_dram[MEM_SENSORS_ACCEL_Z]) - 16384; // Testing is done at 1g, full scale is +-2g, subtract theoretical 1g
		gyro_x_bias += (int32_t)pru1_dram[MEM_SENSORS_GYRO_X];
		gyro_y_bias += (int32_t)pru1_dram[MEM_SENSORS_GYRO_Y];
		gyro_z_bias += (int32_t)pru1_dram[MEM_SENSORS_GYRO_Z];
		usleep(50000);
	}
	
	accel_x_bias >>= 7;
	accel_y_bias >>= 7;
	accel_z_bias >>= 7;
	gyro_x_bias >>= 7;
	gyro_y_bias >>= 7;
	gyro_z_bias >>= 7;
	
	while(1)
	{
		// Reset state variables
		SEq_1 = 1;
		SEq_2 = 0;
		SEq_3 = 0;
		SEq_4 = 0; 
		b_x = 1;
		b_z = 0; 
		w_bx = 0;
		w_by = 0; 
		w_bz = 0;
		
		data[3] = 0;
		
		while(data[3] != 1)
		{
			float accel_x = (float)(((int32_t)pru1_dram[MEM_SENSORS_ACCEL_X]) - accel_x_bias);
			float accel_y = (float)(((int32_t)pru1_dram[MEM_SENSORS_ACCEL_Y]) - accel_y_bias);
			float accel_z = (float)(((int32_t)pru1_dram[MEM_SENSORS_ACCEL_Z]) - accel_z_bias);
			float gyro_x = (float)(((int32_t)pru1_dram[MEM_SENSORS_GYRO_X]) - gyro_x_bias);
			float gyro_y = (float)(((int32_t)pru1_dram[MEM_SENSORS_GYRO_Y]) - gyro_y_bias);
			float gyro_z = (float)(((int32_t)pru1_dram[MEM_SENSORS_GYRO_Z]) - gyro_z_bias);
			float mag_x = (float)((int32_t)pru1_dram[MEM_SENSORS_MAG_X_C]);
			float mag_y = (float)((int32_t)pru1_dram[MEM_SENSORS_MAG_Y_C]);
			float mag_z = (float)((int32_t)pru1_dram[MEM_SENSORS_MAG_Z_C]);
			
			/*
			printf(
				"XL:%3.1f %3.1f %3.1f\t G:%3.1f %3.1f %3.1f\t Mag:%3.1f %3.1f %3.1f\t",
				accel_x,
				accel_y,
				accel_z,
				gyro_x,
				gyro_y,
				gyro_z,
				mag_x,
				mag_y,
				mag_z
				);
			*/
			/*
			printf(
				"XL:%3.1f %d %d\t G:%d %d %d\t Mag:%d %d %d\t",
				(float)((int32_t)pru1_dram[MEM_SENSORS_ACCEL_X] - accel_x_bias),
				(int32_t)pru1_dram[MEM_SENSORS_ACCEL_Y],
				(int32_t)pru1_dram[MEM_SENSORS_ACCEL_Z],
				(int32_t)pru1_dram[MEM_SENSORS_GYRO_X],
				(int32_t)pru1_dram[MEM_SENSORS_GYRO_Y],
				(int32_t)pru1_dram[MEM_SENSORS_GYRO_Z],
				(int32_t)pru1_dram[MEM_SENSORS_MAG_X_C],
				(int32_t)pru1_dram[MEM_SENSORS_MAG_Y_C],
				(int32_t)pru1_dram[MEM_SENSORS_MAG_Z_C]
				);
			*/
			gyro_x *= (500.0/32768.0)*(M_PI/180.0);
			gyro_y *= (500.0/32768.0)*(M_PI/180.0);
			gyro_z *= (500.0/32768.0)*(M_PI/180.0);
					
			filterUpdate(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z);
			
			// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
			float roll = atan2(2*(SEq_1*SEq_2 + SEq_3*SEq_4), 1 - 2*(SEq_2*SEq_2 + SEq_3*SEq_3));
			float pitch = asin(2*(SEq_1*SEq_3 - SEq_4*SEq_2));
			float yaw = atan2(2*(SEq_1*SEq_4 + SEq_2*SEq_3), 1 - 2*(SEq_3*SEq_3 + SEq_4*SEq_4));
			
			data[0] = roll*180.0/M_PI;
			data[1] = pitch*180.0/M_PI;
			data[2] = yaw*180.0/M_PI;
			
			//printf("%3.1f\n", yaw*180.0/M_PI);
			
			sleep_until(&ts,20*1000*1000); // Note: Delay in ns
		}
	}
	
      
    //detach from shared memory  
    shmdt(data); 
  
    return 0; 
} 

// Adds "delay" nanoseconds to timespecs and sleeps until that time
void sleep_until(struct timespec *ts, int delay)
{
        
        ts->tv_nsec += delay;
        if(ts->tv_nsec >= 1000*1000*1000){
                ts->tv_nsec -= 1000*1000*1000;
                ts->tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts,  NULL);
}

// Function to compute one filter iteration
void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z)
{
	// local system variables
	float norm; // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
	float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
	J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
	float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
	float h_x, h_y, h_z; // computed flux in the earth frame
	// axulirary variables to avoid reapeated calcualtions
	float halfSEq_1 = 0.5f * SEq_1;
	float halfSEq_2 = 0.5f * SEq_2;
	float halfSEq_3 = 0.5f * SEq_3;
	float halfSEq_4 = 0.5f * SEq_4;
	float twoSEq_1 = 2.0f * SEq_1;
	float twoSEq_2 = 2.0f * SEq_2;
	float twoSEq_3 = 2.0f * SEq_3;
	float twoSEq_4 = 2.0f * SEq_4;
	float twob_x = 2.0f * b_x;
	float twob_z = 2.0f * b_z;
	float twob_xSEq_1 = 2.0f * b_x * SEq_1;
	float twob_xSEq_2 = 2.0f * b_x * SEq_2;
	float twob_xSEq_3 = 2.0f * b_x * SEq_3;
	float twob_xSEq_4 = 2.0f * b_x * SEq_4;
	float twob_zSEq_1 = 2.0f * b_z * SEq_1;
	float twob_zSEq_2 = 2.0f * b_z * SEq_2;
	float twob_zSEq_3 = 2.0f * b_z * SEq_3;
	float twob_zSEq_4 = 2.0f * b_z * SEq_4;
	float SEq_1SEq_2;
	float SEq_1SEq_3 = SEq_1 * SEq_3;
	float SEq_1SEq_4;
	float SEq_2SEq_3;
	float SEq_2SEq_4 = SEq_2 * SEq_4;
	float SEq_3SEq_4;
	float twom_x = 2.0f * m_x;
	float twom_y = 2.0f * m_y;
	float twom_z = 2.0f * m_z;
	// normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;
	// normalise the magnetometer measurement
	norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
	m_x /= norm;
	m_y /= norm;
	m_z /= norm;
	// compute the objective function and Jacobian
	f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
	f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
	f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
	f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
	f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
	f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;
	J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
	J_12or23 = 2.0f * SEq_4;
	J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21; // negated in matrix multiplication
	J_33 = 2.0f * J_11or24; // negated in matrix multiplication
	J_41 = twob_zSEq_3; // negated in matrix multiplication
	J_42 = twob_zSEq_4;
	J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
	J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
	J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
	J_52 = twob_xSEq_3 + twob_zSEq_1;
	J_53 = twob_xSEq_2 + twob_zSEq_4;
	J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
	J_61 = twob_xSEq_3;
	J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
	J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
	J_64 = twob_xSEq_2;
	// compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
	// normalise the gradient to estimate direction of the gyroscope error
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 = SEqHatDot_1 / norm;
	SEqHatDot_2 = SEqHatDot_2 / norm;

	SEqHatDot_3 = SEqHatDot_3 / norm;
	SEqHatDot_4 = SEqHatDot_4 / norm;
	// compute angular estimated direction of the gyroscope error
	w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
	w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
	w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
	// compute and remove the gyroscope baises
	w_bx += w_err_x * deltat * zeta;
	w_by += w_err_y * deltat * zeta;
	w_bz += w_err_z * deltat * zeta;
	w_x -= w_bx;
	w_y -= w_by;
	w_z -= w_bz;
	// compute the quaternion rate measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
	// compute then integrate the estimated quaternion rate
	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
	// normalise quaternion
	norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
	SEq_1 /= norm;
	SEq_2 /= norm;
	SEq_3 /= norm;
	SEq_4 /= norm;
	// compute flux in the earth frame
	SEq_1SEq_2 = SEq_1 * SEq_2; // recompute axulirary variables
	SEq_1SEq_3 = SEq_1 * SEq_3;
	SEq_1SEq_4 = SEq_1 * SEq_4;
	SEq_3SEq_4 = SEq_3 * SEq_4;
	SEq_2SEq_3 = SEq_2 * SEq_3;
	SEq_2SEq_4 = SEq_2 * SEq_4;
	h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
	h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
	h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);
	// normalise the flux vector to have only components in the x and z
	b_x = sqrt((h_x * h_x) + (h_y * h_y));
	b_z = h_z;
}