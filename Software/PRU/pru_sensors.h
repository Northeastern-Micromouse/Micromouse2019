/**
 * pru_sensors.h
 * Various definitions and declarations for the PRU sensor management system.
 */
#ifndef PRU_SENSORS_H
#define PRU_SENSORS_H

#define MEM_RDY		0x00 // System ready flag memory location
#define MEM_ARM_RDY_VALUE 0x01 // Value that will be placed in memory by ARM side to signal start
#define MEM_RDY_VALUE 0x02 // Value that will be in memory when system is ready

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

#endif
