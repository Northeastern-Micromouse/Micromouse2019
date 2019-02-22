/**
 * pru_sensors.c
 * IMU and distance sensor management system for the TI PRU
 *
 */
 
#include <stdint.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <sys_tscAdcSs.h>
#include <rsc_types.h>
#include "resource_table_empty.h"
#include "pru_sensors.h"
#include "imu.h"
#include "adc.h"

#define PRU1_DRAM	0x00000	// Offset to DRAM
// Skip the first 0x200 byte of DRAM since the Makefile allocates
// 0x100 for the STACK and 0x100 for the HEAP.
volatile unsigned int *shared_memory = (unsigned int *) (PRU1_DRAM + 0x200);

volatile register uint32_t __R30;
volatile register uint32_t __R31;

/* Control Module registers to enable the ADC peripheral */
#define CM_WKUP_CLKSTCTRL  (*((volatile unsigned int *)0x44E00400))
#define CM_WKUP_ADC_TSC_CLKCTRL  (*((volatile unsigned int *)0x44E004BC))

// ADC Function Prototypes
void init_adc(void);
uint16_t read_adc(uint8_t);

// IMU Function Prototypes
void read_accel_gyro(uint8_t, uint8_t*, uint8_t);
void write_accel_gyro(uint8_t, uint8_t*, uint8_t);
void read_mag(uint8_t, uint8_t*, uint8_t);
void write_mag(uint8_t, uint8_t*, uint8_t);

void init_mag(void);
void init_accel_gyro(void);

void check_ready(void);
void read_imu(void);

void main(void)
{
	/* Clear SYSCFG[STANDBY_INIT] to enable OCP master port */
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;
	
	// Wait for the host to trigger initialization
	shared_memory[MEM_SENSORS_RDY] = 0;
	while(shared_memory[MEM_SENSORS_RDY] == 0);
	
	init_mag();
	init_accel_gyro();
	init_adc();
	
	// Once the hose says it's ready, check for the IMU being ready every 100ms
	while(shared_memory[MEM_SENSORS_RDY] == MEM_SENSORS_ARM_RDY_VALUE)
	{
		check_ready();
		__delay_cycles(20000000);
	}
	
	while(1)
	{
		shared_memory[MEM_AIN0] = read_adc(0);
		shared_memory[MEM_AIN1] = read_adc(1);
		shared_memory[MEM_AIN2] = read_adc(2);
		shared_memory[MEM_AIN3] = read_adc(3);
		shared_memory[MEM_AIN4] = read_adc(4);
		shared_memory[MEM_AIN5] = read_adc(5);
		shared_memory[MEM_AIN6] = read_adc(6);
		read_imu();
	}
}

void check_ready()
{
	uint8_t mag_ready = 0;
	uint8_t accel_gyro_ready = 0;
	
	read_mag(WHO_AM_I_M, &mag_ready, 1);
	read_accel_gyro(WHO_AM_I_XG, &accel_gyro_ready, 1);
	
	if(mag_ready == WHO_AM_I_M_RSP && accel_gyro_ready == WHO_AM_I_AG_RSP)
	{
		shared_memory[MEM_RDY] = MEM_RDY_VALUE;
	}
}

void read_imu()
{
	uint8_t mag_data[6] = {0};
	uint8_t accel_data[6] = {0};
	uint8_t gyro_data[6] = {0};
	
	read_mag(OUT_X_L_M, mag_data, 6);
	read_accel_gyro(OUT_X_L_XL, accel_data, 6);
	read_accel_gyro(OUT_X_L_G, gyro_data, 6);
	
	int16_t mag_x = mag_data[1];
	mag_x = (mag_x << 8) | mag_data[0];
	shared_memory[MEM_MAG_X] = mag_x;
	
	int16_t mag_y = mag_data[3];
	mag_y = (mag_y << 8) | mag_data[2];
	shared_memory[MEM_MAG_Y] = mag_y;
	
	int16_t mag_z = mag_data[5];
	mag_z = (mag_z << 8) | mag_data[4];
	shared_memory[MEM_MAG_Z] = mag_z;
	
	int16_t accel_x = accel_data[1];
	accel_x = (accel_x << 8) | accel_data[0];
	shared_memory[MEM_ACCEL_X] = accel_x;
	
	int16_t accel_y = accel_data[3];
	accel_y = (accel_y << 8) | accel_data[2];
	shared_memory[MEM_ACCEL_Y] = accel_y;
	
	int16_t accel_z = accel_data[5];
	accel_z = (accel_z << 8) | accel_data[4];
	shared_memory[MEM_ACCEL_Z] = accel_z;
	
	int16_t gyro_x = gyro_data[1];
	gyro_x = (gyro_x << 8) | gyro_data[0];
	shared_memory[MEM_GYRO_X] = gyro_x;
	
	int16_t gyro_y = gyro_data[3];
	gyro_y = (gyro_y << 8) | gyro_data[2];
	shared_memory[MEM_GYRO_Y] = gyro_y;
	
	int16_t gyro_z = gyro_data[5];
	gyro_z = (gyro_z << 8) | gyro_data[4];
	shared_memory[MEM_GYRO_Z] = gyro_z;
}

/**
 * void read_accel_gyro(uint8_t addr, uint8_t* data, uint8_t len)
 * Shifts data into the processor from the accel/gyro.
 */
void read_accel_gyro(uint8_t addr, uint8_t* data, uint8_t len)
{
	int byte_count;
	int bit_count;
	
	// Bring CS low
	IMU_CSAG_LOW();
	IMU_SH_DELAY();
	
	// We are reading, shift out a 1
	IMU_SCK_LOW();
	IMU_SCK_DELAY();
	
	IMU_DIN_HIGH();
	IMU_SH_DELAY();
	
	IMU_SCK_HIGH();
	IMU_SCK_DELAY();
	
	// Shift out the remaining 7 bits of the address, MSB first
	for(bit_count = 0; bit_count < 7; bit_count++)
	{
		IMU_SCK_LOW();
		IMU_SCK_DELAY();
		
		if(addr & (1 << (6-bit_count)))
		{
			IMU_DIN_HIGH();
		}
		else
		{
			IMU_DIN_LOW();
		}
		IMU_SH_DELAY();
		
		IMU_SCK_HIGH();
		IMU_SCK_DELAY();
	}
	
	for(byte_count = 0; byte_count < len; byte_count++)
	{
		data[byte_count] = 0;
		
		for(bit_count = 0; bit_count < 8; bit_count++)
		{
			IMU_SCK_LOW();
			IMU_SCK_DELAY();
			
			if(IMU_READ_SDOAG())
			{
				data[byte_count] |= (1 << (7-bit_count));
			}
			
			IMU_SCK_HIGH();
			IMU_SCK_DELAY();
		}
	}
	
	// Finally, de-select the device
	IMU_CSAG_HIGH();
}

/**
 * void write_accel_gyro(uint8_t addr, uint8_t* data, uint8_t len)
 * Shifts data out of the processdor into the accel/gyro.
 */
void write_accel_gyro(uint8_t addr, uint8_t* data, uint8_t len)
{
	int byte_count;
	int bit_count;
	
	// Bring CS low
	IMU_CSAG_LOW();
	IMU_SH_DELAY();
	
	// We are writing, shift out a 0
	IMU_SCK_LOW();
	IMU_SCK_DELAY();
	
	IMU_DIN_LOW();
	IMU_SH_DELAY();
	
	IMU_SCK_HIGH();
	IMU_SCK_DELAY();
	
	// Shift out the remaining 7 bits of the address, MSB first
	for(bit_count = 0; bit_count < 7; bit_count++)
	{
		IMU_SCK_LOW();
		IMU_SCK_DELAY();
		
		if(addr & (1 << (6-bit_count)))
		{
			IMU_DIN_HIGH();
		}
		else
		{
			IMU_DIN_LOW();
		}
		IMU_SH_DELAY();
		
		IMU_SCK_HIGH();
		IMU_SCK_DELAY();
	}
	
	for(byte_count = 0; byte_count < len; byte_count++)
	{
		for(bit_count = 0; bit_count < 8; bit_count++)
		{
			IMU_SCK_LOW();
			IMU_SCK_DELAY();
			
			if(data[byte_count] & (1 << (7-bit_count)))
			{
				IMU_DIN_HIGH();
			}
			else
			{
				IMU_DIN_LOW();
			}
			IMU_SH_DELAY();
			
			IMU_SCK_HIGH();
			IMU_SCK_DELAY();
		}
	}
	
	// Finally, de-select the device
	IMU_CSAG_HIGH();
}

/**
 * void read_mag(uint8_t addr, uint8_t* data, uint8_t len)
 * Shifts data into the processor from the magnetometer.
 */
void read_mag(uint8_t addr, uint8_t* data, uint8_t len)
{
	int byte_count;
	int bit_count;
	
	// Bring CS low
	IMU_CSM_LOW();
	IMU_SH_DELAY();
	
	// We are reading, shift out a 1
	IMU_SCK_LOW();
	IMU_SCK_DELAY();
	
	IMU_DIN_HIGH();
	IMU_SH_DELAY();
	
	IMU_SCK_HIGH();
	IMU_SCK_DELAY();
	
	// Shift out the remaining 7 bits of the address, MSB first
	for(bit_count = 0; bit_count < 7; bit_count++)
	{
		IMU_SCK_LOW();
		IMU_SCK_DELAY();
		
		if(addr & (1 << (6-bit_count)))
		{
			IMU_DIN_HIGH();
		}
		else
		{
			IMU_DIN_LOW();
		}
		IMU_SH_DELAY();
		
		IMU_SCK_HIGH();
		IMU_SCK_DELAY();
	}
	
	for(byte_count = 0; byte_count < len; byte_count++)
	{
		data[byte_count] = 0;
		
		for(bit_count = 0; bit_count < 8; bit_count++)
		{
			IMU_SCK_LOW();
			IMU_SCK_DELAY();
			
			if(IMU_READ_SDOM())
			{
				data[byte_count] |= (1 << (7-bit_count));
			}
			
			IMU_SCK_HIGH();
			IMU_SCK_DELAY();
		}
	}
	
	// Finally, de-select the device
	IMU_CSM_HIGH();
}

/**
 * void write_accel_gyro(uint8_t addr, uint8_t* data, uint8_t len)
 * Shifts data out of the processor into the magnetometer.
 */
void write_mag(uint8_t addr, uint8_t* data, uint8_t len)
{
	int byte_count;
	int bit_count;
	
	// Bring CS low
	IMU_CSM_LOW();
	IMU_SH_DELAY();
	
	// We are writing, shift out a 0
	IMU_SCK_LOW();
	IMU_SCK_DELAY();
	
	IMU_DIN_LOW();
	IMU_SH_DELAY();
	
	IMU_SCK_HIGH();
	IMU_SCK_DELAY();
	
	// Shift out the remaining 7 bits of the address, MSB first
	for(bit_count = 0; bit_count < 7; bit_count++)
	{
		IMU_SCK_LOW();
		IMU_SCK_DELAY();
		
		if(addr & (1 << (6-bit_count)))
		{
			IMU_DIN_HIGH();
		}
		else
		{
			IMU_DIN_LOW();
		}
		IMU_SH_DELAY();
		
		IMU_SCK_HIGH();
		IMU_SCK_DELAY();
	}
	
	for(byte_count = 0; byte_count < len; byte_count++)
	{
		for(bit_count = 0; bit_count < 8; bit_count++)
		{
			IMU_SCK_LOW();
			IMU_SCK_DELAY();
			
			if(data[byte_count] & (1 << (7-bit_count)))
			{
				IMU_DIN_HIGH();
			}
			else
			{
				IMU_DIN_LOW();
			}
			IMU_SH_DELAY();
			
			IMU_SCK_HIGH();
			IMU_SCK_DELAY();
		}
	}
	
	// Finally, de-select the device
	IMU_CSM_HIGH();
}

void init_accel_gyro()
{
	// Soft reset
	uint8_t ctrl_reg8 = 0x05;
	write_accel_gyro(CTRL_REG8, &ctrl_reg8, 1);
	__delay_cycles(200000000);
	
	// FS_G = 01 = 0x01   (500 dps range) 
	// ODR_G = 110 = 0x06 (952Hz ODR, 100Hz LPF cutoff) 
	// BW_G = 11 = 0x03   (100Hz LPF cutoff)
	uint8_t ctrl_reg1_g = (0x01 << 3) | (0x6 << 5) | 0x03;
	write_accel_gyro(CTRL_REG1_G, &ctrl_reg1_g, 1);

	// ODR_XL = 110 = 0x06 (952kHz ODR, no power-down)
	// FS_XL = 0 = 0x00 (+/-2g range)
	// BW_SCAL_ODR = 0x00 (bandwidth determined by ODR selection)
	uint8_t ctrl_reg6_xl = (0x06 << 5);
	write_accel_gyro(CTRL_REG6_XL, &ctrl_reg6_xl, 1);
	
	// Use only defaults for ctrl_reg7_xl
	// This means: HR = 0, FDS = 0, HPIS1 = 0
	
	// Wait 100ms
	__delay_cycles(20000000);
}

void init_mag()
{
	// Soft reset
	uint8_t ctrl_reg2_m = (1 << 2);
	write_mag(CTRL_REG2_M, &ctrl_reg2_m, 1);
	__delay_cycles(200000000);
	
	// Write the calibration offset
	/*
	uint8_t offsets[] = 
	{
		shared_memory[MEM_MAG_OFFSET_X] & 0xFF,
		(shared_memory[MEM_MAG_OFFSET_X] >> 8) & 0xFF,
		shared_memory[MEM_MAG_OFFSET_Y] & 0xFF,
		(shared_memory[MEM_MAG_OFFSET_Y] >> 8) & 0xFF,
		shared_memory[MEM_MAG_OFFSET_Z] & 0xFF,
		(shared_memory[MEM_MAG_OFFSET_Z] >> 8) & 0xFF
	};
	*/
	uint8_t offsets[6] = {0}; 
	
	write_mag(OFFSET_X_REG_L_M, offsets, 6);
	
	// OM = 11 = 0x03 (XY ultra-high performance)
	// DO = 111 = 0x07 (80Hz ODR)
	uint8_t ctrl_reg1_m = (0x03 << 5) | (0x07 << 2);
	write_mag(CTRL_REG1_M, &ctrl_reg1_m, 1);
	
	// MD = 00 = 0x00 (continuous conversion mode)
	uint8_t ctrl_reg3_m = 0x00;
	write_mag(CTRL_REG3_M, &ctrl_reg3_m, 1);
	
	// OMZ = 11 = 0x03 (Z ultra-high performance)
	uint8_t ctrl_reg4_m = (0x03 << 2);
	write_mag(CTRL_REG4_M, &ctrl_reg4_m, 1);
	
	// Wait 100ms
	__delay_cycles(20000000);
}

void init_adc()
{

	/* set the always on clock domain to NO_SLEEP. Enable ADC_TSC clock */
    while (!(CM_WKUP_ADC_TSC_CLKCTRL == 0x02))
	{
		CM_WKUP_CLKSTCTRL = 0;
		CM_WKUP_ADC_TSC_CLKCTRL = 0x02;
		/* Optional: implement timeout logic. */
    }

    /*
     * Set the ADC_TSC CTRL register.
     * Disable TSC_ADC_SS module so we can program it.
     * Set step configuration registers to writable.
     */
    ADC_TSC.CTRL_bit.ENABLE = 0;
    ADC_TSC.CTRL_bit.STEPCONFIG_WRITEPROTECT_N_ACTIVE_LOW = 1;

    /*
     * set the ADC_TSC STEPCONFIG1 register for channel 0
     * Mode = 0; SW enabled, one-shot
     * Averaging = 0x3; 8 sample average
     * SEL_INP_SWC_3_0 = 0x0 = Channel 0
     * SEL_INM_SWC_3_0 = 1xxx = VREFN (reduces noise in single ended mode)
     * use FIFO0
     */
    ADC_TSC.STEPCONFIG1_bit.MODE = 0;
    ADC_TSC.STEPCONFIG1_bit.AVERAGING = 3;
    ADC_TSC.STEPCONFIG1_bit.SEL_INP_SWC_3_0 = 0;
    ADC_TSC.STEPCONFIG1_bit.SEL_INM_SWC_3_0 = 8;
    ADC_TSC.STEPCONFIG1_bit.FIFO_SELECT = 0;

    /*
     * set the ADC_TSC STEPCONFIG2 register for channel 1
     * Mode = 0; SW enabled, one-shot
     * Averaging = 0x3; 8 sample average
     * SEL_INP_SWC_3_0 = 0x1 = Channel 1
     * SEL_INM_SWC_3_0 = 1xxx = VREFN (reduces noise in single ended mode)
     * use FIFO0
     */
    ADC_TSC.STEPCONFIG2_bit.MODE = 0;
    ADC_TSC.STEPCONFIG2_bit.AVERAGING = 3;
    ADC_TSC.STEPCONFIG2_bit.SEL_INP_SWC_3_0 = 1;
    ADC_TSC.STEPCONFIG2_bit.SEL_INM_SWC_3_0 = 8;
    ADC_TSC.STEPCONFIG2_bit.FIFO_SELECT = 0;

    /*
     * set the ADC_TSC STEPCONFIG3 register for channel 2
     * Mode = 0; SW enabled, one-shot
     * Averaging = 0x3; 8 sample average
     * SEL_INP_SWC_3_0 = 0x2 = Channel 2
     * SEL_INM_SWC_3_0 = 1xxx = VREFN (reduces noise in single ended mode)
     * use FIFO0
     */
    ADC_TSC.STEPCONFIG3_bit.MODE = 0;
    ADC_TSC.STEPCONFIG3_bit.AVERAGING = 3;
    ADC_TSC.STEPCONFIG3_bit.SEL_INP_SWC_3_0 = 2;
    ADC_TSC.STEPCONFIG3_bit.SEL_INM_SWC_3_0 = 8;
    ADC_TSC.STEPCONFIG3_bit.FIFO_SELECT = 0;

    /*
     * set the ADC_TSC STEPCONFIG4 register for channel 3
     * Mode = 0; SW enabled, one-shot
     * Averaging = 0x3; 8 sample average
     * SEL_INP_SWC_3_0 = 0x3 = Channel 3
     * SEL_INM_SWC_3_0 = 1xxx = VREFN (reduces noise in single ended mode)
     * use FIFO0
     */
    ADC_TSC.STEPCONFIG4_bit.MODE = 0;
    ADC_TSC.STEPCONFIG4_bit.AVERAGING = 3;
    ADC_TSC.STEPCONFIG4_bit.SEL_INP_SWC_3_0 = 3;
    ADC_TSC.STEPCONFIG4_bit.SEL_INM_SWC_3_0 = 8;
    ADC_TSC.STEPCONFIG4_bit.FIFO_SELECT = 0;
	
	/*
     * set the ADC_TSC STEPCONFIG5 register for channel 4
     * Mode = 0; SW enabled, one-shot
     * Averaging = 0x3; 8 sample average
     * SEL_INP_SWC_3_0 = 0x4 = Channel 4
     * SEL_INM_SWC_3_0 = 1xxx = VREFN (reduces noise in single ended mode)
     * use FIFO0
     */
    ADC_TSC.STEPCONFIG5_bit.MODE = 0;
    ADC_TSC.STEPCONFIG5_bit.AVERAGING = 3;
    ADC_TSC.STEPCONFIG5_bit.SEL_INP_SWC_3_0 = 4;
    ADC_TSC.STEPCONFIG5_bit.SEL_INM_SWC_3_0 = 8;
    ADC_TSC.STEPCONFIG5_bit.FIFO_SELECT = 0;
	
	/*
     * set the ADC_TSC STEPCONFIG5 register for channel 5
     * Mode = 0; SW enabled, one-shot
     * Averaging = 0x3; 8 sample average
     * SEL_INP_SWC_3_0 = 0x5 = Channel 5
     * SEL_INM_SWC_3_0 = 1xxx = VREFN (reduces noise in single ended mode)
     * use FIFO0
     */
    ADC_TSC.STEPCONFIG6_bit.MODE = 0;
    ADC_TSC.STEPCONFIG6_bit.AVERAGING = 3;
    ADC_TSC.STEPCONFIG6_bit.SEL_INP_SWC_3_0 = 5;
    ADC_TSC.STEPCONFIG6_bit.SEL_INM_SWC_3_0 = 8;
    ADC_TSC.STEPCONFIG6_bit.FIFO_SELECT = 0;
	
	/*
     * set the ADC_TSC STEPCONFIG5 register for channel 6
     * Mode = 0; SW enabled, one-shot
     * Averaging = 0x3; 8 sample average
     * SEL_INP_SWC_3_0 = 0x6 = Channel 6
     * SEL_INM_SWC_3_0 = 1xxx = VREFN (reduces noise in single ended mode)
     * use FIFO0
     */
    ADC_TSC.STEPCONFIG7_bit.MODE = 0;
    ADC_TSC.STEPCONFIG7_bit.AVERAGING = 3;
    ADC_TSC.STEPCONFIG7_bit.SEL_INP_SWC_3_0 = 6;
    ADC_TSC.STEPCONFIG7_bit.SEL_INM_SWC_3_0 = 8;
    ADC_TSC.STEPCONFIG7_bit.FIFO_SELECT = 0;

    /*
     * set the ADC_TSC CTRL register
     * set step configuration registers to protected
     * store channel ID tag if needed for debug
     * Enable TSC_ADC_SS module
     */
    ADC_TSC.CTRL_bit.STEPCONFIG_WRITEPROTECT_N_ACTIVE_LOW = 0;
    ADC_TSC.CTRL_bit.STEP_ID_TAG = 1;
    ADC_TSC.CTRL_bit.ENABLE = 1;
}

uint16_t read_adc(uint8_t channel)
{
    /*
     * Clear FIFO0 by reading from it
     * We are using single-shot mode.
     * It should not usually enter the for loop
     */
    uint32_t count = ADC_TSC.FIFO0COUNT;

    /* read from the specified ADC channel */
    switch (channel) {
		case 0:
		  ADC_TSC.STEPENABLE_bit.STEP1 = 1;
		case 1:
		  ADC_TSC.STEPENABLE_bit.STEP2 = 1;
		case 2:
		  ADC_TSC.STEPENABLE_bit.STEP3 = 1;
		case 3:
		  ADC_TSC.STEPENABLE_bit.STEP4 = 1;
		case 4:
		  ADC_TSC.STEPENABLE_bit.STEP5 = 1;
		case 5:
		  ADC_TSC.STEPENABLE_bit.STEP6 = 1;
		case 6:
		  ADC_TSC.STEPENABLE_bit.STEP7 = 1;
		default :
			break;
    }

    while (ADC_TSC.FIFO0COUNT == 0) {
      /*
       * loop until value placed in fifo.
       * Optional: implement timeout logic.
       *
       * Other potential options include:
       *   - configure PRU to receive an ADC interrupt. Note that
       *     END_OF_SEQUENCE does not mean that the value has been
       *     loaded into the FIFO yet
       *   - perform other actions, and periodically poll for
       *     FIFO0COUNT > 0
       */
    }

    /* read the voltage */
    uint16_t voltage = ADC_TSC.FIFO0DATA_bit.ADCDATA;

    return voltage;
}
