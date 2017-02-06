// estop.h
// Contains all code required to control two set of ear lights.

// This file is part of Dreamer Head v0.4.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.22

#define RUNNING 0
#define STOPPED 1

void eStopInit(void);
uint32_t hardStopStatus(void);
void softRun(void);
void softStop(void);
uint32_t softStopStatus(void);
