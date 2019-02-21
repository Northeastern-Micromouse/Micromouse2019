/**
 * adc.h
 * Reads from the ADC inside the PRU subsystem.
 */
 
#include <stdint.h>

void init_adc(void);
uint16_t read_adc(uint8_t);
