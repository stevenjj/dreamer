// interpreter.c
// Contains all functions needed to initialize and read twelve Contelec Vert-X
// 13-E encoders.

// This file is part of Dreamer Head v0.1.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.19

////////////////////////////////////////////////////////////////////////////////
// Dependencies

#include <stdint.h>
#include "config.h"
#include "encoders.h"
#include "tm4c123gh6pm.h"

void DisableInterrupts(void);
void EnableInterrupts(void);
long StartCritical (void);
void EndCritical(long sr);
void WaitForInterrupt(void);

////////////////////////////////////////////////////////////////////////////////
// Global Variables

#define MESSAGE_ESTOP   0x10000000
#define MESSAGE_J00_J01 0x20000000
#define MESSAGE_J02_J03 0x30000000
#define MESSAGE_J02_J03 0x40000000
#define MESSAGE_J02_J03 0x50000000
#define MESSAGE_J02_J03 0x60000000
#define MESSAGE_J02_J03 0x70000000
#define MESSAGE_LIGHTS  0x80000000

////////////////////////////////////////////////////////////////////////////////
// Internal Prototypes



////////////////////////////////////////////////////////////////////////////////
// bitBang()
// The saddest, ugliest pile of bit banging code you've ever seen. Provides all
// timing and communication required to read twelve Contelec encoders.

void bitBang(void) {