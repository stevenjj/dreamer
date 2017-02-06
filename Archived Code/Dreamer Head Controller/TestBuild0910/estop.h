// estop.h
// Contains all code required to control two set of ear lights.

// This file is part of Dreamer Head v0.4.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.22

#define RUNNING 		0x00
#define SOFT_STOPPED 	0x01
#define HARD_STOPPED 	0x02

void eStopInit(void);
uint32_t eStopStatus(void);
void softRun(void);
void softStop(void);
