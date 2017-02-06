// control.c
// Contains all functions for control systems for Dreamer's head.

// This file is part of Dreamer Head v0.1.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.18

////////////////////////////////////////////////////////////////////////////////
// Dependencies

#include <stdint.h>
#include "config.h"
#include "control.h"
#include "tm4c123gh6pm.h"

void DisableInterrupts(void);
void EnableInterrupts(void);
long StartCritical (void);
void EndCritical(long sr);

// void (*loopTask)(void);

////////////////////////////////////////////////////////////////////////////////
// Internal Prototypes

void controlInit(void);// (*newFunc)(void), uint32_t *inArray, uint32_t *outArray);

////////////////////////////////////////////////////////////////////////////////
// controlInit()

void controlInit(void) {// (*newFunc)(void), uint32_t *inArray, uint32_t *outArray) {
    // loopTask = newFunc;
    // posn = inArray;
    // ctrl = outArray;
}
