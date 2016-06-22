// encoders.c
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

uint32_t
    dataA00, dataB00, dataD00, dataE00, dataA01, dataB01, dataD01, dataE01,
    dataA02, dataB02, dataD02, dataE02, dataA03, dataB03, dataD03, dataE03, 
    dataA04, dataB04, dataD04, dataE04, dataA05, dataB05, dataD05, dataE05, 
    dataA06, dataB06, dataD06, dataE06, dataA07, dataB07, dataD07, dataE07, 
    dataA08, dataB08, dataD08, dataE08, dataA09, dataB09, dataD09, dataE09, 
    dataA10, dataB10, dataD10, dataE10, dataA11, dataB11, dataD11, dataE11, 
    dataA12, dataB12, dataD12, dataE12, dataA13, dataB13, dataD13, dataE13, 
    dataA14, dataB14, dataD14, dataE14, dataA15, dataB15, dataD15, dataE15;

////////////////////////////////////////////////////////////////////////////////
// Internal Prototypes

void PortA_Init(void);
void PortB_Init(void);
void PortD_Init(void);
void PortE_Init(void);
void wait(uint32_t num);

////////////////////////////////////////////////////////////////////////////////
// encoderInit()
// Initializes all GPIO pins needed to read twelve joint encoders.

void encoderInit(void) {
    PortA_Init();
    PortB_Init();
    PortD_Init();
    PortE_Init();
}

////////////////////////////////////////////////////////////////////////////////
// encoderRead()
// The saddest, ugliest pile of bit banging code you've ever seen. Provides all
// timing and communication required to read twelve Contelec encoders at 550Hz.

void encoderRead(void) {
    volatile uint32_t spin = 0;

    // Start encoder message
    encSS = 0x0000;

    // Write Byte 1
    // Tick 01
    encCl = 0xFFFF;
    enc00 = 0xFFFF; enc01 = 0xFFFF; enc02 = 0xFFFF; enc03 = 0xFFFF;
    enc04 = 0xFFFF; enc05 = 0xFFFF; enc06 = 0xFFFF; enc07 = 0xFFFF;
    enc08 = 0xFFFF; enc09 = 0xFFFF; enc10 = 0xFFFF; enc11 = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 02
    encCl = 0xFFFF;
    enc00 = 0x0000; enc01 = 0x0000; enc02 = 0x0000; enc03 = 0x0000;
    enc04 = 0x0000; enc05 = 0x0000; enc06 = 0x0000; enc07 = 0x0000;
    enc08 = 0x0000; enc09 = 0x0000; enc10 = 0x0000; enc11 = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 03
    encCl = 0xFFFF;
    enc00 = 0xFFFF; enc01 = 0xFFFF; enc02 = 0xFFFF; enc03 = 0xFFFF;
    enc04 = 0xFFFF; enc05 = 0xFFFF; enc06 = 0xFFFF; enc07 = 0xFFFF;
    enc08 = 0xFFFF; enc09 = 0xFFFF; enc10 = 0xFFFF; enc11 = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 04
    encCl = 0xFFFF;
    enc00 = 0x0000; enc01 = 0x0000; enc02 = 0x0000; enc03 = 0x0000;
    enc04 = 0x0000; enc05 = 0x0000; enc06 = 0x0000; enc07 = 0x0000;
    enc08 = 0x0000; enc09 = 0x0000; enc10 = 0x0000; enc11 = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 05
    encCl = 0xFFFF;
    enc00 = 0xFFFF; enc01 = 0xFFFF; enc02 = 0xFFFF; enc03 = 0xFFFF;
    enc04 = 0xFFFF; enc05 = 0xFFFF; enc06 = 0xFFFF; enc07 = 0xFFFF;
    enc08 = 0xFFFF; enc09 = 0xFFFF; enc10 = 0xFFFF; enc11 = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 06
    encCl = 0xFFFF;
    enc00 = 0x0000; enc01 = 0x0000; enc02 = 0x0000; enc03 = 0x0000;
    enc04 = 0x0000; enc05 = 0x0000; enc06 = 0x0000; enc07 = 0x0000;
    enc08 = 0x0000; enc09 = 0x0000; enc10 = 0x0000; enc11 = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 07
    encCl = 0xFFFF;
    enc00 = 0xFFFF; enc01 = 0xFFFF; enc02 = 0xFFFF; enc03 = 0xFFFF;
    enc04 = 0xFFFF; enc05 = 0xFFFF; enc06 = 0xFFFF; enc07 = 0xFFFF;
    enc08 = 0xFFFF; enc09 = 0xFFFF; enc10 = 0xFFFF; enc11 = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 08
    encCl = 0xFFFF;
    enc00 = 0x0000; enc01 = 0x0000; enc02 = 0x0000; enc03 = 0x0000;
    enc04 = 0x0000; enc05 = 0x0000; enc06 = 0x0000; enc07 = 0x0000;
    enc08 = 0x0000; enc09 = 0x0000; enc10 = 0x0000; enc11 = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;

    wait(140);  // inter-byte delay

    // Write Byte 2
    // Tick 01
    encCl = 0xFFFF;
    enc00 = 0xFFFF; enc01 = 0xFFFF; enc02 = 0xFFFF; enc03 = 0xFFFF;
    enc04 = 0xFFFF; enc05 = 0xFFFF; enc06 = 0xFFFF; enc07 = 0xFFFF;
    enc08 = 0xFFFF; enc09 = 0xFFFF; enc10 = 0xFFFF; enc11 = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 02
    encCl = 0xFFFF;
    enc00 = 0xFFFF; enc01 = 0xFFFF; enc02 = 0xFFFF; enc03 = 0xFFFF;
    enc04 = 0xFFFF; enc05 = 0xFFFF; enc06 = 0xFFFF; enc07 = 0xFFFF;
    enc08 = 0xFFFF; enc09 = 0xFFFF; enc10 = 0xFFFF; enc11 = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
		// Tick 03
    encCl = 0xFFFF;
    enc00 = 0xFFFF; enc01 = 0xFFFF; enc02 = 0xFFFF; enc03 = 0xFFFF;
    enc04 = 0xFFFF; enc05 = 0xFFFF; enc06 = 0xFFFF; enc07 = 0xFFFF;
    enc08 = 0xFFFF; enc09 = 0xFFFF; enc10 = 0xFFFF; enc11 = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 04
    encCl = 0xFFFF;
    enc00 = 0xFFFF; enc01 = 0xFFFF; enc02 = 0xFFFF; enc03 = 0xFFFF;
    enc04 = 0xFFFF; enc05 = 0xFFFF; enc06 = 0xFFFF; enc07 = 0xFFFF;
    enc08 = 0xFFFF; enc09 = 0xFFFF; enc10 = 0xFFFF; enc11 = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
		// Tick 05
    encCl = 0xFFFF;
    enc00 = 0xFFFF; enc01 = 0xFFFF; enc02 = 0xFFFF; enc03 = 0xFFFF;
    enc04 = 0xFFFF; enc05 = 0xFFFF; enc06 = 0xFFFF; enc07 = 0xFFFF;
    enc08 = 0xFFFF; enc09 = 0xFFFF; enc10 = 0xFFFF; enc11 = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 06
    encCl = 0xFFFF;
    enc00 = 0xFFFF; enc01 = 0xFFFF; enc02 = 0xFFFF; enc03 = 0xFFFF;
    enc04 = 0xFFFF; enc05 = 0xFFFF; enc06 = 0xFFFF; enc07 = 0xFFFF;
    enc08 = 0xFFFF; enc09 = 0xFFFF; enc10 = 0xFFFF; enc11 = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
		// Tick 07
    encCl = 0xFFFF;
    enc00 = 0xFFFF; enc01 = 0xFFFF; enc02 = 0xFFFF; enc03 = 0xFFFF;
    enc04 = 0xFFFF; enc05 = 0xFFFF; enc06 = 0xFFFF; enc07 = 0xFFFF;
    enc08 = 0xFFFF; enc09 = 0xFFFF; enc10 = 0xFFFF; enc11 = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 08
    encCl = 0xFFFF;
    enc00 = 0xFFFF; enc01 = 0xFFFF; enc02 = 0xFFFF; enc03 = 0xFFFF;
    enc04 = 0xFFFF; enc05 = 0xFFFF; enc06 = 0xFFFF; enc07 = 0xFFFF;
    enc08 = 0xFFFF; enc09 = 0xFFFF; enc10 = 0xFFFF; enc11 = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
		enc00 = 0x0000; enc01 = 0x0000; enc02 = 0x0000; enc03 = 0x0000;
    enc04 = 0x0000; enc05 = 0x0000; enc06 = 0x0000; enc07 = 0x0000;
    enc08 = 0x0000; enc09 = 0x0000; enc10 = 0x0000; enc11 = 0x0000;

    wait(140);  // inter-byte delay

    // change all pins to read
    GPIO_PORTA_DIR_R &= ~0x0C;
    GPIO_PORTB_DIR_R &= ~0x0F;
    GPIO_PORTD_DIR_R &= ~0xCC;
    GPIO_PORTE_DIR_R &= ~0x0F;

    // Read Byte 3
    // Tick 01
    encCl = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    dataA15 = GPIO_PORTA_DATA_R;
    dataB15 = GPIO_PORTB_DATA_R;
    dataD15 = GPIO_PORTD_DATA_R;
    dataE15 = GPIO_PORTE_DATA_R;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 02
    encCl = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    dataA14 = GPIO_PORTA_DATA_R;
    dataB14 = GPIO_PORTB_DATA_R;
    dataD14 = GPIO_PORTD_DATA_R;
    dataE14 = GPIO_PORTE_DATA_R;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 03
    encCl = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    dataA13 = GPIO_PORTA_DATA_R;
    dataB13 = GPIO_PORTB_DATA_R;
    dataD13 = GPIO_PORTD_DATA_R;
    dataE13 = GPIO_PORTE_DATA_R;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 04
    encCl = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    dataA12 = GPIO_PORTA_DATA_R;
    dataB12 = GPIO_PORTB_DATA_R;
    dataD12 = GPIO_PORTD_DATA_R;
    dataE12 = GPIO_PORTE_DATA_R;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 05
    encCl = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    dataA11 = GPIO_PORTA_DATA_R;
    dataB11 = GPIO_PORTB_DATA_R;
    dataD11 = GPIO_PORTD_DATA_R;
    dataE11 = GPIO_PORTE_DATA_R;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 06
    encCl = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    dataA10 = GPIO_PORTA_DATA_R;
    dataB10 = GPIO_PORTB_DATA_R;
    dataD10 = GPIO_PORTD_DATA_R;
    dataE10 = GPIO_PORTE_DATA_R;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 07
    encCl = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    dataA09 = GPIO_PORTA_DATA_R;
    dataB09 = GPIO_PORTB_DATA_R;
    dataD09 = GPIO_PORTD_DATA_R;
    dataE09 = GPIO_PORTE_DATA_R;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 08
    encCl = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    dataA08 = GPIO_PORTA_DATA_R;
    dataB08 = GPIO_PORTB_DATA_R;
    dataD08 = GPIO_PORTD_DATA_R;
    dataE08 = GPIO_PORTE_DATA_R;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;

    wait(140);  // inter-byte delay

    // Read Byte 4
    // Tick 01
    encCl = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    dataA07 = GPIO_PORTA_DATA_R;
    dataB07 = GPIO_PORTB_DATA_R;
    dataD07 = GPIO_PORTD_DATA_R;
    dataE07 = GPIO_PORTE_DATA_R;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 02
    encCl = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    dataA06 = GPIO_PORTA_DATA_R;
    dataB06 = GPIO_PORTB_DATA_R;
    dataD06 = GPIO_PORTD_DATA_R;
    dataE06 = GPIO_PORTE_DATA_R;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 03
    encCl = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    dataA05 = GPIO_PORTA_DATA_R;
    dataB05 = GPIO_PORTB_DATA_R;
    dataD05 = GPIO_PORTD_DATA_R;
    dataE05 = GPIO_PORTE_DATA_R;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 04
    encCl = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    dataA04 = GPIO_PORTA_DATA_R;
    dataB04 = GPIO_PORTB_DATA_R;
    dataD04 = GPIO_PORTD_DATA_R;
    dataE04 = GPIO_PORTE_DATA_R;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 05
    encCl = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    dataA03 = GPIO_PORTA_DATA_R;
    dataB03 = GPIO_PORTB_DATA_R;
    dataD03 = GPIO_PORTD_DATA_R;
    dataE03 = GPIO_PORTE_DATA_R;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 06
    encCl = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    dataA02 = GPIO_PORTA_DATA_R;
    dataB02 = GPIO_PORTB_DATA_R;
    dataD02 = GPIO_PORTD_DATA_R;
    dataE02 = GPIO_PORTE_DATA_R;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 07
    encCl = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    dataA01 = GPIO_PORTA_DATA_R;
    dataB01 = GPIO_PORTB_DATA_R;
    dataD01 = GPIO_PORTD_DATA_R;
    dataE01 = GPIO_PORTE_DATA_R;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Tick 08
    encCl = 0xFFFF;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    encCl = 0x0000;
    dataA00 = GPIO_PORTA_DATA_R;
    dataB00 = GPIO_PORTB_DATA_R;
    dataD00 = GPIO_PORTD_DATA_R;
    dataE00 = GPIO_PORTE_DATA_R;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;

    // change all pins to write
    GPIO_PORTA_DIR_R |= 0x0C;
    GPIO_PORTB_DIR_R |= 0x0F;
    GPIO_PORTD_DIR_R |= 0xCC;
    GPIO_PORTE_DIR_R |= 0x0F;
		
		// End encoder message
		encCl = 0x0000;
    enc00 = 0x0000; enc01 = 0x0000; enc02 = 0x0000; enc03 = 0x0000;
    enc04 = 0x0000; enc05 = 0x0000; enc06 = 0x0000; enc07 = 0x0000;
    enc08 = 0x0000; enc09 = 0x0000; enc10 = 0x0000; enc11 = 0x0000;
    encSS = 0xFFFF;

}

////////////////////////////////////////////////////////////////////////////////
// PortA_Init()
// Initializes all required GPIO pins on Port A.

void PortA_Init(void) {
    DisableInterrupts();
        SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
                                            // activate port A
        while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R0) == 0) {}
                                            // wait for clock to stabilize
        GPIO_PORTA_DIR_R |= 0x3C;           // make PA2-5 output
        GPIO_PORTA_AFSEL_R &= ~0x3C;        // disable alt funct on PA2-5
        GPIO_PORTA_DEN_R |= 0x3C;           // enable digital I/O on PA2-5
        GPIO_PORTA_PCTL_R &= ~0x00FFFF00;   // configure PA2-5 as GPIO
        GPIO_PORTA_AMSEL_R &= ~0x3C;        // disable analog funcs on PA2-5
    EnableInterrupts();
}

////////////////////////////////////////////////////////////////////////////////
// PortB_Init()
// Initializes all required GPIO pins on Port B.

void PortB_Init(void) {
    DisableInterrupts();
        SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
                                            // activate port B
        while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R1) == 0) {}
                                            // wait for clock to stabilize
        GPIO_PORTB_DIR_R |= 0x0F;           // make PB0-3 output
        GPIO_PORTB_AFSEL_R &= ~0x0F;        // disable alt funct on PB0-3
        GPIO_PORTB_DEN_R |= 0x0F;           // enable digital I/O on PB0-3
        GPIO_PORTB_PCTL_R &= ~0x0000FFFF;   // configure PB0-3 as GPIO
        GPIO_PORTB_AMSEL_R &= ~0x0F;        // disable analog funcs on PB0-3
    EnableInterrupts();
}

////////////////////////////////////////////////////////////////////////////////
// PortD_Init()
// Initializes all required GPIO pins on Port D.

void PortD_Init(void) {
    DisableInterrupts();
        SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
                                            // activate port D
        while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R3) == 0) {}
                                            // wait for clock to stabilize
        GPIO_PORTD_LOCK_R |= GPIO_LOCK_KEY; // unlock GPIO Port D Commit Reg
        GPIO_PORTD_CR_R |= 0x80;            // enable commit for PD7
        GPIO_PORTD_DIR_R |= 0xCC;           // make PD2,3,6,7 output
        GPIO_PORTD_AFSEL_R &= ~0xCC;        // disable alt funct on PD2,3,6,7
        GPIO_PORTD_DEN_R |= 0xCC;           // enable digital I/O on PD2,3,6,7
        GPIO_PORTD_PCTL_R &= ~0xFF00FF00;   // configure PD2,3,6,7 as GPIO
        GPIO_PORTD_AMSEL_R &= ~0xCC;        // disable analog funcs on PD2,3,6,7
    EnableInterrupts();
}

////////////////////////////////////////////////////////////////////////////////
// PortE_Init()
// Initializes all required GPIO pins on Port E.

void PortE_Init(void) {
    DisableInterrupts();
        SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
                                            // activate port E
        while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0) {}
                                            // wait for clock to stabilize
        GPIO_PORTE_DIR_R |= 0x0F;           // make PE0-3 output
        GPIO_PORTE_AFSEL_R &= ~0x0F;        // disable alt funct on PE0-3
        GPIO_PORTE_DEN_R |= 0x0F;           // enable digital I/O on PE0-3
        GPIO_PORTE_PCTL_R &= ~0x0000FFFF;   // configure PE0-3 as GPIO
        GPIO_PORTE_AMSEL_R &= ~0x0F;        // disable analog funcs on PE0-3
    EnableInterrupts();
}

void wait(uint32_t num) {
    volatile uint32_t spin = 0;

    for(uint32_t i = 0; i < num; i++)
        spin = spin;
}
