#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdlib.h>

#define MAXCH 4

#define PRU_ADDR        0x4A300000      // Start of PRU memory Page 184 am335x TRM
#define PRU_LEN         0x80000         // Length of PRU memory
#define PRU0_DRAM       0x00000         // Offset to DRAM
#define PRU1_DRAM       0x02000
#define PRU_SHAREDMEM   0x10000         // Offset to shared memory

unsigned int    *pru0DRAM_32int_ptr;        // Points to the start of local DRAM
unsigned int    *prusharedMem_32int_ptr;    // Points to the start of the shared memory

uint32_t stepsLeft, usLeft, stepsRight, usRight;

int main(int argc, char *argv[])
{
  unsigned int    *pru;       // Points to start of PRU memory.
  int fd;
  //printf("Servo tester\n");

  fd = open ("/dev/mem", O_RDWR | O_SYNC);
  if (fd == -1) {
    printf ("ERROR: could not open /dev/mem.\n\n");
    return 1;
  }
  pru = mmap (0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDR);
  if (pru == MAP_FAILED) {
    printf ("ERROR: could not map memory.\n\n");
    return 1;
  }
  close(fd);
  printf ("Using /dev/mem.\n");

  pru0DRAM_32int_ptr =     pru + PRU0_DRAM/4 + 0x200/4;   // Points to 0x200 of PRU0 memory
  //prusharedMem_32int_ptr = pru + PRU_SHAREDMEM/4; // Points to start of shared memory

  unsigned int *pruDRAM_32int_ptr = pru0DRAM_32int_ptr;
  pruDRAM_32int_ptr[2] = 2500; //period
  pruDRAM_32int_ptr[4] = 2500; //right period
  pruDRAM_32int_ptr[1] = 400;
  pruDRAM_32int_ptr[3] = 400;
  int x;
  printf("Open loop");
  while (pruDRAM_32int_ptr[1] && pruDRAM_32int_ptr[3]) {}
  printf("Exit loop");
  if(munmap(pru, PRU_LEN)) {
    printf("munmap failed\n");
  } else {
    printf("munmap succeeded\n");
  }


}
