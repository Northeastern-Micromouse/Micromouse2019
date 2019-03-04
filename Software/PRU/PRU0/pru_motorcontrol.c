/**
 * pru_motorcontrol.c
 * Motor control system for PRU
 */
 
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include "resource_table_empty.h"
#include "pru_motorcontrol.h"

// Hardware step pins
#define MM_MOTOR_LEFT	(1 << 0)
#define MM_MOTOR_RIGHT	(1 << 2)

#define PRU0_DRAM   0x00000        // Offset to DRAM

volatile register uint32_t __R30;
volatile register uint32_t __R31;

volatile unsigned int *shared_memory = (unsigned int *) (PRU0_DRAM + 0x200);

void main(void)
{

	/* Clear SYSCFG[STANDBY_INIT] to enable OCP master port */
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	shared_memory = (unsigned int *) (PRU0_DRAM + 0x200);
	
	// Wait for the host to trigger initialization
	shared_memory[MEM_MOTORS_RDY] = 0;
	while(shared_memory[MEM_MOTORS_RDY] == 0);

	// Once the host says it's ready, respond and continue
	while(shared_memory[MEM_MOTORS_RDY] != MEM_MOTORS_ARM_RDY_VALUE)
	{
		__delay_cycles(20000000);
	}

	shared_memory[MEM_MOTORS_RDY] = MEM_MOTORS_RDY_VALUE;

	shared_memory[MEM_MOTORS_STEPS_LEFT] = 0;
	shared_memory[MEM_MOTORS_STEPS_RIGHT] = 0;

	for(;;)
	{
		if(shared_memory[MEM_MOTORS_STEPS_LEFT] > 0 && shared_memory[MEM_MOTORS_STEPS_RIGHT] > 0)
		{
			// Multiply by two to convert number of steps into number of
			// GPIO toggles
			shared_memory[MEM_MOTORS_STEPS_LEFT] *= 2;
			shared_memory[MEM_MOTORS_STEPS_RIGHT] *= 2;
			
			int leftCounter = shared_memory[MEM_MOTORS_PERIOD_LEFT];
			int rightCounter = shared_memory[MEM_MOTORS_PERIOD_RIGHT];
			
			// Start stepping
			// While there are remaining steps, loop every half microsecond
			// If it's time to toggle, do so. Otherwise decrement the
			// counter and continue
			while(shared_memory[MEM_MOTORS_STEPS_LEFT] > 0 || shared_memory[MEM_MOTORS_STEPS_RIGHT] > 0)
			{
				if(shared_memory[MEM_MOTORS_STEPS_LEFT] && !leftCounter)
				{
					__R30 ^= MM_MOTOR_LEFT;
					shared_memory[MEM_MOTORS_STEPS_LEFT]--;
					leftCounter = shared_memory[MEM_MOTORS_PERIOD_LEFT];
				}
				if(shared_memory[MEM_MOTORS_STEPS_RIGHT] && !rightCounter)
				{
					__R30 ^= MM_MOTOR_RIGHT;
					shared_memory[MEM_MOTORS_STEPS_RIGHT]--;
					rightCounter = shared_memory[MEM_MOTORS_PERIOD_RIGHT];
				}

				leftCounter--;
				rightCounter--;

				__delay_cycles(100);
			}
		}
	}
}
