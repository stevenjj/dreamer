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
#include "UART.h"

void DisableInterrupts(void);
void EnableInterrupts(void);
long StartCritical (void);
void EndCritical(long sr);
void WaitForInterrupt(void);

////////////////////////////////////////////////////////////////////////////////
// Global Variables

#define CMD_LEN 10
#define PREFIX_MASK		0xF0000000
#define MESSAGE_MASK	0x0FFFFFFF
#define MESSAGE_ESTOP   0x10000000
#define MESSAGE_J00_J01 0x20000000
#define MESSAGE_J02_J03 0x30000000
#define MESSAGE_J04_J05 0x40000000
#define MESSAGE_J06_J07 0x50000000
#define MESSAGE_J08_J09 0x60000000
#define MESSAGE_J10_J11 0x70000000
#define MESSAGE_LIGHTS  0x80000000
#define ESTOP_RUN       0x0532AC00

////////////////////////////////////////////////////////////////////////////////
// Internal Prototypes

void interpreterInit(void);
void parse(void);

////////////////////////////////////////////////////////////////////////////////
// interpreterInit()
// Initialize UART channel and any variables needed for sending/receiving.

void interpreterInit(void) {
	// Initialize UART channel, in and out
	// ensure that in/out queues are empty
} 

////////////////////////////////////////////////////////////////////////////////
// parse()
// Collect next received word from UART queue. Initiate action it requests.

void parse(void) {
	char cmd[CMD_LEN+1];

	UART_InStringNL(cmd, CMD_LEN);

	uint32_t value = cmd;
	uint32_t prefix = value&PREFIX_MASK;
	value &= MESSAGE_MASK;

	switch(address) {
		case MESSAGE_ESTOP:
			if(value == ESTOP_RUN)
				softRun();
			else
				softStop();
			break;
		case MESSAGE_J00_J01:
			desrPos[0] = value/16383;
			desrPos[1] = value%16384;
			break;
		case MESSAGE_J00_J01:
			desrPos[2] = value/16383;
			desrPos[3] = value%16384;
			break;
		case MESSAGE_J00_J01:
			desrPos[4] = value/16383;
			desrPos[5] = value%16384;
			break;
		case MESSAGE_J00_J01:
			desrPos[6] = value/16383;
			desrPos[7] = value%16384;
			break;
		case MESSAGE_J00_J01:
			desrPos[8] = value/16383;
			desrPos[9] = value%16384;
			break;
		case MESSAGE_J00_J01:
			desrPos[10] = value/16383;
			desrPos[11] = value%16384;
			break;
		case MESSAGE_LIGHTS:
			color = value;
			break;
	}
}