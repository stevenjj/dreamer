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

int32_t *position;
uint32_t
    dataB02, dataB03, dataB04, dataB05, dataB06, dataB07, dataB08, dataB09,
    dataB10, dataB11, dataB12, dataB13, dataB14, dataB15,
    dataD02, dataD03, dataD04, dataD05, dataD06, dataD07, dataD08, dataD09,
    dataD10, dataD11, dataD12, dataD13, dataD14, dataD15,
    dataE02, dataE03, dataE04, dataE05, dataE06, dataE07, dataE08, dataE09,
    dataE10, dataE11, dataE12, dataE13, dataE14, dataE15;
uint32_t new00, new01, new02, new03, new04, new05, new06, new07, new08, new09,
    new10, new11;

////////////////////////////////////////////////////////////////////////////////
// Internal Prototypes

void bitBang(void);
void filter(void);
uint32_t median(uint32_t *data);
void PortA_Init(void);
void PortB_Init(void);
void PortD_Init(void);
void PortE_Init(void);
void transform(void);
void wait(uint32_t num);

////////////////////////////////////////////////////////////////////////////////
// bitBang()
// The saddest, ugliest pile of bit banging code you've ever seen. Provides all
// timing and communication required to read twelve Contelec encoders.

void bitBang(void) {
    volatile uint32_t spin = 0;

    // Start encoder message
    encSS = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    // Compensating for the fact that SS pin drops low much more slowly
    // than all other pins. Need to find out why that is. Pullup still in
    // place?
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;

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

    wait(150);  // inter-byte delay

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

    wait(150);  // inter-byte delay

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
    dataB15 = GPIO_PORTB_DATA_R;
    dataD15 = GPIO_PORTD_DATA_R;
    dataE15 = GPIO_PORTE_DATA_R;
    new00 = (dataB15&0x01)<<13;
    new01 = (dataB15&0x02)<<12;
    new02 = (dataB15&0x04)<<11;
    new03 = (dataB15&0x08)<<10;
    
    // Tick 02
    encCl = 0xFFFF;
    new04 = (dataD15&0x04)<<11;
    new05 = (dataD15&0x08)<<10;
    new06 = (dataD15&0x40)<<7;
    new07 = (dataD15&0x80)<<6;
    new08 = (dataE15&0x01)<<13;
    new09 = (dataE15&0x02)<<12;
    new10 = (dataE15&0x04)<<11;
    new11 = (dataE15&0x08)<<10;
    encCl = 0x0000;
    dataB14 = GPIO_PORTB_DATA_R;
    dataD14 = GPIO_PORTD_DATA_R;
    dataE14 = GPIO_PORTE_DATA_R;
    new00 += (dataB14&0x01)<<12;
    new01 += (dataB14&0x02)<<11;
    new02 += (dataB14&0x04)<<10;
    new03 += (dataB14&0x08)<<9;
    // Tick 03
    encCl = 0xFFFF;
    new04 += (dataD14&0x04)<<10;
    new05 += (dataD14&0x08)<<9;
    new06 += (dataD14&0x40)<<6;
    new07 += (dataD14&0x80)<<5;
    new08 += (dataE14&0x01)<<12;
    new09 += (dataE14&0x02)<<11;
    new10 += (dataE14&0x04)<<10;
    new11 += (dataE14&0x08)<<9;
    encCl = 0x0000;
    dataB13 = GPIO_PORTB_DATA_R;
    dataD13 = GPIO_PORTD_DATA_R;
    dataE13 = GPIO_PORTE_DATA_R;
    new00 += (dataB13&0x01)<<11;
    new01 += (dataB13&0x02)<<10;
    new02 += (dataB13&0x04)<<9;
    new03 += (dataB13&0x08)<<8;
    // Tick 04
    encCl = 0xFFFF;
    new04 += (dataD13&0x04)<<9;
    new05 += (dataD13&0x08)<<8;
    new06 += (dataD13&0x40)<<5;
    new07 += (dataD13&0x80)<<4;
    new08 += (dataE13&0x01)<<11;
    new09 += (dataE13&0x02)<<10;
    new10 += (dataE13&0x04)<<9;
    new11 += (dataE13&0x08)<<8;
    encCl = 0x0000;
    dataB12 = GPIO_PORTB_DATA_R;
    dataD12 = GPIO_PORTD_DATA_R;
    dataE12 = GPIO_PORTE_DATA_R;
    new00 += (dataB12&0x01)<<10;
    new01 += (dataB12&0x02)<<9;
    new02 += (dataB12&0x04)<<8;
    new03 += (dataB12&0x08)<<7;
    // Tick 05
    encCl = 0xFFFF;
    new04 += (dataD12&0x04)<<8;
    new05 += (dataD12&0x08)<<7;
    new06 += (dataD12&0x40)<<4;
    new07 += (dataD12&0x80)<<3;
    new08 += (dataE12&0x01)<<10;
    new09 += (dataE12&0x02)<<9;
    new10 += (dataE12&0x04)<<8;
    new11 += (dataE12&0x08)<<7;
    encCl = 0x0000;
    dataB11 = GPIO_PORTB_DATA_R;
    dataD11 = GPIO_PORTD_DATA_R;
    dataE11 = GPIO_PORTE_DATA_R;
    new00 += (dataB11&0x01)<<9;
    new01 += (dataB11&0x02)<<8;
    new02 += (dataB11&0x04)<<7;
    new03 += (dataB11&0x08)<<6;
    // Tick 06
    encCl = 0xFFFF;
    new04 += (dataD11&0x04)<<7;
    new05 += (dataD11&0x08)<<6;
    new06 += (dataD11&0x40)<<3;
    new07 += (dataD11&0x80)<<2;
    new08 += (dataE11&0x01)<<9;
    new09 += (dataE11&0x02)<<8;
    new10 += (dataE11&0x04)<<7;
    new11 += (dataE11&0x08)<<6;
    encCl = 0x0000;
    dataB10 = GPIO_PORTB_DATA_R;
    dataD10 = GPIO_PORTD_DATA_R;
    dataE10 = GPIO_PORTE_DATA_R;
    new00 += (dataB10&0x01)<<8;
    new01 += (dataB10&0x02)<<7;
    new02 += (dataB10&0x04)<<6;
    new03 += (dataB10&0x08)<<5;
    // Tick 07
    encCl = 0xFFFF;
    new04 += (dataD10&0x04)<<6;
    new05 += (dataD10&0x08)<<5;
    new06 += (dataD10&0x40)<<2;
    new07 += (dataD10&0x80)<<1;
    new08 += (dataE10&0x01)<<8;
    new09 += (dataE10&0x02)<<7;
    new10 += (dataE10&0x04)<<6;
    new11 += (dataE10&0x08)<<5;
    encCl = 0x0000;
    dataB09 = GPIO_PORTB_DATA_R;
    dataD09 = GPIO_PORTD_DATA_R;
    dataE09 = GPIO_PORTE_DATA_R;
    new00 += (dataB09&0x01)<<7;
    new01 += (dataB09&0x02)<<6;
    new02 += (dataB09&0x04)<<5;
    new03 += (dataB09&0x08)<<4;
    // Tick 08
    encCl = 0xFFFF;
    new04 += (dataD09&0x04)<<5;
    new05 += (dataD09&0x08)<<4;
    new06 += (dataD09&0x40)<<1;
    new07 += dataD09&0x80;
    new08 += (dataE09&0x01)<<7;
    new09 += (dataE09&0x02)<<6;
    new10 += (dataE09&0x04)<<5;
    new11 += (dataE09&0x08)<<4;
    encCl = 0x0000;
    dataB08 = GPIO_PORTB_DATA_R;
    dataD08 = GPIO_PORTD_DATA_R;
    dataE08 = GPIO_PORTE_DATA_R;
    new00 += (dataB08&0x01)<<6;
    new01 += (dataB08&0x02)<<5;
    new02 += (dataB08&0x04)<<4;
    new03 += (dataB08&0x08)<<3;

    wait(150);  // inter-byte delay

    // Read Byte 4
    // Tick 01
    encCl = 0xFFFF;
    new04 += (dataD08&0x04)<<4;
    new05 += (dataD08&0x08)<<3;
    new06 += dataD08&0x40;
    new07 += (dataD08&0x80)>>1;
    new08 += (dataE08&0x01)<<6;
    new09 += (dataE08&0x02)<<5;
    new10 += (dataE08&0x04)<<4;
    new11 += (dataE08&0x08)<<3;
    encCl = 0x0000;
    dataB07 = GPIO_PORTB_DATA_R;
    dataD07 = GPIO_PORTD_DATA_R;
    dataE07 = GPIO_PORTE_DATA_R;
    new00 += (dataB07&0x01)<<5;
    new01 += (dataB07&0x02)<<4;
    new02 += (dataB07&0x04)<<3;
    new03 += (dataB07&0x08)<<2;
    // Tick 02
    encCl = 0xFFFF;
    new04 += (dataD07&0x04)<<3;
    new05 += (dataD07&0x08)<<2;
    new06 += (dataD07&0x40)>>1;
    new07 += (dataD07&0x80)>>2;
    new08 += (dataE07&0x01)<<5;
    new09 += (dataE07&0x02)<<4;
    new10 += (dataE07&0x04)<<3;
    new11 += (dataE07&0x08)<<2;
    encCl = 0x0000;
    dataB06 = GPIO_PORTB_DATA_R;
    dataD06 = GPIO_PORTD_DATA_R;
    dataE06 = GPIO_PORTE_DATA_R;
    new00 += (dataB06&0x01)<<4;
    new01 += (dataB06&0x02)<<3;
    new02 += (dataB06&0x04)<<2;
    new03 += (dataB06&0x08)<<1;
    // Tick 03
    encCl = 0xFFFF;
    new04 += (dataD06&0x04)<<2;
    new05 += (dataD06&0x08)<<1;
    new06 += (dataD06&0x40)>>2;
    new07 += (dataD06&0x80)>>3;
    new08 += (dataE06&0x01)<<4;
    new09 += (dataE06&0x02)<<3;
    new10 += (dataE06&0x04)<<2;
    new11 += (dataE06&0x08)<<1;
    encCl = 0x0000;
    dataB05 = GPIO_PORTB_DATA_R;
    dataD05 = GPIO_PORTD_DATA_R;
    dataE05 = GPIO_PORTE_DATA_R;
    new00 += (dataB05&0x01)<<3;
    new01 += (dataB05&0x02)<<2;
    new02 += (dataB05&0x04)<<1;
    new03 += dataB05&0x08;
    // Tick 04
    encCl = 0xFFFF;
    new04 += (dataD05&0x04)<<1;
    new05 += dataD05&0x08;
    new06 += (dataD05&0x40)>>3;
    new07 += (dataD05&0x80)>>4;
    new08 += (dataE05&0x01)<<3;
    new09 += (dataE05&0x02)<<2;
    new10 += (dataE05&0x04)<<1;
    new11 += dataE05&0x08;
    encCl = 0x0000;
    dataB04 = GPIO_PORTB_DATA_R;
    dataD04 = GPIO_PORTD_DATA_R;
    dataE04 = GPIO_PORTE_DATA_R;
    new00 += (dataB04&0x01)<<2;
    new01 += (dataB04&0x02)<<1;
    new02 += dataB04&0x04;
    new03 += (dataB04&0x08)>>1;
    // Tick 05
    encCl = 0xFFFF;
    new04 += dataD04&0x04;
    new05 += (dataD04&0x08)>>1;
    new06 += (dataD04&0x40)>>4;
    new07 += (dataD04&0x80)>>5;
    new08 += (dataE04&0x01)<<2;
    new09 += (dataE04&0x02)<<1;
    new10 += dataE04&0x04;
    new11 += (dataE04&0x08)>>1;
    encCl = 0x0000;
    dataB03 = GPIO_PORTB_DATA_R;
    dataD03 = GPIO_PORTD_DATA_R;
    dataE03 = GPIO_PORTE_DATA_R;
    new00 += (dataB03&0x01)<<1;
    new01 += dataB03&0x02;
    new02 += (dataB03&0x04)>>1;
    new03 += (dataB03&0x08)>>2;
    // Tick 06
    encCl = 0xFFFF;
    new04 += (dataD03&0x04)>>1;
    new05 += (dataD03&0x08)>>2;
    new06 += (dataD03&0x40)>>5;
    new07 += (dataD03&0x80)>>6;
    new08 += (dataE03&0x01)<<1;
    new09 += dataE03&0x02;
    new10 += (dataE03&0x04)>>1;
    new11 += (dataE03&0x08)>>2;
    encCl = 0x0000;
    dataB02 = GPIO_PORTB_DATA_R;
    dataD02 = GPIO_PORTD_DATA_R;
    dataE02 = GPIO_PORTE_DATA_R;
    new00 += dataB02&0x01;
    new01 += (dataB02&0x02)>>1;
    new02 += (dataB02&0x04)>>2;
    new03 += (dataB02&0x08)>>3;
    // Tick 07
    encCl = 0xFFFF;
    new04 += (dataD02&0x04)>>2;
    new05 += (dataD02&0x08)>>3;
    new06 += (dataD02&0x40)>>6;
    new07 += (dataD02&0x80)>>7;
    new08 += dataE02&0x01;
    new09 += (dataE02&0x02)>>1;
    new10 += (dataE02&0x04)>>2;
    new11 += (dataE02&0x08)>>3;
    encCl = 0x0000;
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
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
    spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;spin=0;
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
// encoderInit()
// Initializes all GPIO pins needed to read twelve joint encoders.

void encoderInit(int32_t *newArray) {
    position = newArray;

    PortA_Init();
    PortB_Init();
    PortD_Init();
    PortE_Init();
}

////////////////////////////////////////////////////////////////////////////////
// encoderRead()
// All actions required to read and filter one encoder reading from twelve
// Contelec encoders in parallel.

void encoderRead(void) {
    bitBang();
    transform();
    filter();
}

////////////////////////////////////////////////////////////////////////////////
// filter()
// Filter new encoder readings as desired.

void filter(void) {
    static uint32_t last = 0;
    static uint32_t filt00[3] = {0};
    static uint32_t filt01[3] = {0};
    static uint32_t filt02[3] = {0};
    static uint32_t filt03[3] = {0};
    static uint32_t filt04[3] = {0};
    static uint32_t filt05[3] = {0};
    static uint32_t filt06[3] = {0};
    static uint32_t filt07[3] = {0};
    static uint32_t filt08[3] = {0};
    static uint32_t filt09[3] = {0};
    static uint32_t filt10[3] = {0};
    static uint32_t filt11[3] = {0};
    
    filt00[last] = position[0];
    filt01[last] = position[1];
    filt02[last] = position[2];
    filt03[last] = position[3];
    filt04[last] = position[4];
    filt05[last] = position[5];
    filt06[last] = position[6];
    filt07[last] = position[7];
    filt08[last] = position[8];
    filt09[last] = position[9];
    filt10[last] = position[10];
    filt11[last] = position[11];
    last = (last + 1)%3;
    
    position[0] = median(filt00);
    position[1] = median(filt01);
    position[2] = median(filt02);
    position[3] = median(filt03);
    position[4] = median(filt04);
    position[5] = median(filt05);
    position[6] = median(filt06);
    position[7] = median(filt07);
    position[8] = median(filt08);
    position[9] = median(filt09);
    position[10] = median(filt10);
    position[11] = median(filt11);
}

////////////////////////////////////////////////////////////////////////////////
// median()
// returns the median of a 3-element array.

uint32_t median(uint32_t *data) {
    uint32_t sort[3] = {0};
    uint32_t temp = 0;
    
    sort[0] = data[0];
    sort[1] = data[1];
    sort[2] = data[2];
    
    if(sort[0] > sort[1]) {
        temp = sort[0];
        sort[0] = sort[1];
        sort[1] = temp;
    }
    if(sort[1] > sort[2]) {
        temp = sort[1];
        sort[1] = sort[2];
        sort[2] = temp;
    }
    if(sort[0] > sort[1]) {
        temp = sort[0];
        sort[0] = sort[1];
        sort[1] = temp;
    }
    
    return sort[1];
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
        GPIO_PORTA_PCTL_R &= ~0x00FFFF00;   // configure PA2-5 as GPIO
        GPIO_PORTA_DEN_R |= 0x3C;           // enable digital I/O on PA2-5
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
        GPIO_PORTB_PCTL_R &= ~0x0000FFFF;   // configure PB0-3 as GPIO
        GPIO_PORTB_DEN_R |= 0x0F;           // enable digital I/O on PB0-3
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
        GPIO_PORTD_PCTL_R &= ~0xFF00FF00;   // configure PD2,3,6,7 as GPIO
        GPIO_PORTD_DEN_R |= 0xCC;           // enable digital I/O on PD2,3,6,7
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
        GPIO_PORTE_PCTL_R &= ~0x0000FFFF;   // configure PE0-3 as GPIO
        GPIO_PORTE_DEN_R |= 0x0F;           // enable digital I/O on PE0-3
        GPIO_PORTE_AMSEL_R &= ~0x0F;        // disable analog funcs on PE0-3
    EnableInterrupts();
}

////////////////////////////////////////////////////////////////////////////////
// transform()
// Transform raw gpio reads into encoder values.

void transform(void) {
    position[0] = new04;
    position[1] = new05;
    position[2] = new06;
    position[3] = new07;
    position[4] = new08;
    position[5] = new02;
    position[6] = new09;
    position[7] = new03;
    position[8] = new11;
    position[9] = new10;
    position[10] = new00;
    position[11] = new01;
}

////////////////////////////////////////////////////////////////////////////////
// wait()
// Wait for an imprecise but large number of cycles.

void wait(uint32_t num) {
    volatile uint32_t spin = 0;

    for(uint32_t i = 0; i < num; i++)
        spin = spin;
}
