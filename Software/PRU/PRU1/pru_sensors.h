/**
 * pru_sensors.h
 * Various definitions and declarations for the PRU sensor management system.
 */
#ifndef PRU_SENSORS_H
#define PRU_SENSORS_H

#define MEM_SENSORS_RDY				0x00 // System ready flag memory location
#define MEM_SENSORS_ARM_RDY_VALUE	0x01 // Value that will be placed in memory by ARM side to signal start
#define MEM_SENSORS_RDY_VALUE		0x02 // Value that will be in memory when system is ready

// Memory locations for raw output values
#define MEM_ACCEL_X	0x01
#define MEM_ACCEL_Y 0x02
#define MEM_ACCEL_Z 0x03
#define MEM_GYRO_X	0x04
#define MEM_GYRO_Y	0x05
#define MEM_GYRO_Z	0x06
#define MEM_MAG_X	0x07
#define MEM_MAG_Y	0x08
#define MEM_MAG_Z	0x09
#define MEM_AIN0	0x0A
#define MEM_AIN1	0x0B
#define MEM_AIN2	0x0C
#define MEM_AIN3	0x0D
#define MEM_AIN4	0x0E
#define MEM_AIN5	0x0F
#define MEM_AIN6	0x10

// Memory locations for computed values
#define MEM_MAGX_C	0x11 // Corrected Mag X
#define MEM_MAGY_C	0x12 // Corrected Mag Y
#define MEM_MAGZ_C	0x13 // Corrected Mag Z
#define MEM_YAW		0x14 // Computed Yaw
#define MEM_PITCH	0x15 // Computed Pitch
#define MEM_ROLL	0x16 // Computed Roll
#define MEM_DIST0	0x17 // Computed distance 0
#define MEM_DIST1	0x18 // Computed distance 1
#define MEM_DIST2	0x19 // Computed distance 2
#define MEM_DIST3	0x1A // Computed distance 3
#define MEM_DIST4	0x1B // Computed distance 4
#define MEM_DIST5	0x1C // Computed distance 5
#define MEM_DIST6	0x1D // Computed distance 6

// Memory locations for calibration data
#define MEM_MAG_OFFSET_X 0x1E
#define MEM_MAG_OFFSET_Y 0x1F
#define MEM_MAG_OFFSET_Z 0x20
#define MEM_MAG_INVW_XX	 0x21
#define MEM_MAG_INVW_XY	 0x22
#define MEM_MAG_INVW_XZ	 0x23
#define MEM_MAG_INVW_YX	 0x24
#define MEM_MAG_INVW_YY	 0x25
#define MEM_MAG_INVW_YZ	 0x26
#define MEM_MAG_INVW_ZX	 0x27
#define MEM_MAG_INVW_ZY	 0x28
#define MEM_MAG_INVW_ZZ	 0x29

// IMU Registers
#define ACT_THS				0x04
#define ACT_DUR				0x05
#define INT_GEN_CFG_XL		0x06
#define INT_GEN_THS_X_XL	0x07
#define INT_GEN_THS_Y_XL	0x08
#define INT_GEN_THS_Z_XL	0x09
#define INT_GEN_DUR_XL		0x0A
#define REFERENCE_G			0x0B
#define INT1_CTRL			0x0C
#define INT2_CTRL			0x0D
#define WHO_AM_I_XG			0x0F
#define CTRL_REG1_G			0x10
#define CTRL_REG2_G			0x11
#define CTRL_REG3_G			0x12
#define ORIENT_CFG_G		0x13
#define INT_GEN_SRC_G		0x14
#define OUT_TEMP_L			0x15
#define OUT_TEMP_H			0x16
#define STATUS_REG_0		0x17
#define OUT_X_L_G			0x18
#define OUT_X_H_G			0x19
#define OUT_Y_L_G			0x1A
#define OUT_Y_H_G			0x1B
#define OUT_Z_L_G			0x1C
#define OUT_Z_H_G			0x1D
#define CTRL_REG4			0x1E
#define CTRL_REG5_XL		0x1F
#define CTRL_REG6_XL		0x20
#define CTRL_REG7_XL		0x21
#define CTRL_REG8			0x22
#define CTRL_REG9			0x23
#define CTRL_REG10			0x24
#define INT_GEN_SRC_XL		0x26
#define STATUS_REG_1		0x27
#define OUT_X_L_XL			0x28
#define OUT_X_H_XL			0x29
#define OUT_Y_L_XL			0x2A
#define OUT_Y_H_XL			0x2B
#define OUT_Z_L_XL			0x2C
#define OUT_Z_H_XL			0x2D
#define FIFO_CTRL			0x2E
#define FIFO_SRC			0x2F
#define INT_GEN_CFG_G		0x30
#define INT_GEN_THS_XH_G	0x31
#define INT_GEN_THS_XL_G	0x32
#define INT_GEN_THS_YH_G	0x33
#define INT_GEN_THS_YL_G	0x34
#define INT_GEN_THS_ZH_G	0x35
#define INT_GEN_THS_ZL_G	0x36
#define INT_GEN_DUR_G		0x37

#define OFFSET_X_REG_L_M	0x05
#define OFFSET_X_REG_H_M	0x06
#define OFFSET_Y_REG_L_M	0x07
#define OFFSET_Y_REG_H_M	0x08
#define OFFSET_Z_REG_L_M	0x09
#define OFFSET_Z_REG_H_M	0x0A
#define WHO_AM_I_M			0x0F
#define CTRL_REG1_M			0x20
#define CTRL_REG2_M			0x21
#define CTRL_REG3_M			0x22
#define CTRL_REG4_M			0x23
#define CTRL_REG5_M			0x24
#define STATUS_REG_M		0x27
#define OUT_X_L_M			0x28
#define OUT_X_H_M			0x29
#define OUT_Y_L_M			0x2A
#define OUT_Y_H_M			0x2B
#define OUT_Z_L_M			0x2C
#define OUT_Z_H_M			0x2D
#define INT_CFG_M			0x30
#define INT_SRC_M			0x30
#define INT_THS_L_M			0x32
#define INT_THS_H_M			0x33

#define WHO_AM_I_AG_RSP		0x68
#define WHO_AM_I_M_RSP		0x3D

// Hardware Constants
#define IMU_DEN_PIN
#define IMU_INT2_PIN
#define IMU_INT1_PIN
#define IMU_INTM_PIN
#define IMU_RDY_PIN
#define IMU_CSM_PIN		3
#define IMU_CSAG_PIN	2
#define IMU_SDOM_PIN	1
#define IMU_SDOAG_PIN	0
#define IMU_DIN_PIN		8
#define IMU_SCK_PIN		10

#define IMU_SCK_LOW()		__R30 &= ~(1 << IMU_SCK_PIN) 
#define IMU_SCK_HIGH()		__R30 |= (1 << IMU_SCK_PIN)
#define IMU_DIN_LOW()		__R30 &= ~(1 << IMU_DIN_PIN)
#define IMU_DIN_HIGH()		__R30 |= (1 << IMU_DIN_PIN)
#define IMU_CSAG_LOW()		__R30 &= ~(1 << IMU_CSAG_PIN)
#define IMU_CSAG_HIGH()		__R30 |= (1 << IMU_CSAG_PIN)
#define IMU_CSM_LOW()		__R30 &= ~(1 << IMU_CSM_PIN)
#define IMU_CSM_HIGH()		__R30 |= (1 << IMU_CSM_PIN)
#define IMU_READ_SDOM()		__R31 & (1 << IMU_SDOM_PIN)
#define IMU_READ_SDOAG()	__R31 & (1 << IMU_SDOAG_PIN)

// IMU Macros
// Each clock cycle is 5ns
// 100ns delay is 20 cycles
#define IMU_SCK_DELAY()	__delay_cycles(20)

// Take care of setup and hold and valid times
#define IMU_SH_DELAY()	__delay_cycles(10)

#endif
