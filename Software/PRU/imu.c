/**
 * imu.c
 * Function definitions for IMU
 */
 
#include "imu.h"

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
	for(bit_count = 0; bit_count < 7; byte_count++)
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
	for(bit_count = 0; bit_count < 7; byte_count++)
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
	for(bit_count = 0; bit_count < 7; byte_count++)
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
	IMU_CSAG_HIGH();
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
	for(bit_count = 0; bit_count < 7; byte_count++)
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
	const uint8_t ctrl_reg8 = 0x05;
	write_accel_gyro(CTRL_REG8, &ctrl_reg8, 1);
	__delay_cycles(200000000);
	
	// FS_G = 01 = 0x01   (500 dps range) 
	// ODR_G = 110 = 0x06 (952Hz ODR, 100Hz LPF cutoff) 
	// BW_G = 11 = 0x03   (100Hz LPF cutoff)
	const uint8_t ctrl_reg1_g = (0x01 << 3) | (0x6 << 5) | 0x03;
	write_accel_gyro(CTRL_REG1_G, &ctrl_reg1_g, 1);

	// ODR_XL = 110 = 0x06 (952kHz ODR, no power-down)
	// FS_XL = 0 = 0x00 (+/-2g range)
	// BW_SCAL_ODR = 0x00 (bandwidth determined by ODR selection)
	const uint8_t ctrl_reg6_xl = (0x06 << 5);
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
	uint8_t[] offsets = 
	{
		shared_mem[MEM_MAG_OFFSET_X] & 0xFF,
		(shared_mem[MEM_MAG_OFFSET_X] >> 8) & 0xFF,
		shared_mem[MEM_MAG_OFFSET_Y] & 0xFF,
		(shared_mem[MEM_MAG_OFFSET_Y] >> 8) & 0xFF,
		shared_mem[MEM_MAG_OFFSET_Z] & 0xFF,
		(shared_mem[MEM_MAG_OFFSET_Z] >> 8) & 0xFF
	};
	
	write_mag(OFFSET_X_REG_L_M, offsets, 6);
	
	// OM = 11 = 0x03 (XY ultra-high performance)
	// DO = 111 = 0x07 (80Hz ODR)
	uint8_t ctrl_reg1_m = (0x03 << 5) | (0x07 << 2s);
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

