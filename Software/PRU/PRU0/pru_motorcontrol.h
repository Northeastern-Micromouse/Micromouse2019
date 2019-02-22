/**
 * pru_motorcontrol.h
 * Motor control system for PRU
 */
 
#ifndef PRU_MOTORCONTROL_H
#define PRU_MOTORCONTROL_H

#define MEM_MOTORS_RDY				0x00 // Memory location for ready flag
#define MEM_MOTORS_ARM_RDY_VALUE	0x01 // Value the ARM will write when ready
#define MEM_MOTORS_RDY_VALUE		0x02 // Value the PRU will respond with when ready

#define MEM_STEPS_LEFT		0x01
#define MEM_STEPS_RIGHT		0x02
#define MEM_PERIOD_LEFT		0x03
#define MEM_PERIOD_RIGHT	0x04

#endif