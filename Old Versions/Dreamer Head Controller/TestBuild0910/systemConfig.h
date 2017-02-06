// sysConfig.h
// This file contains system configuration settings for the Dreamer head. These
// settings should change only if physical hardware changes are made.

// This file is part of Dreamer Head t0910
// Travis Llado, travis@travisllado.com
// Last modified 2016.12.09

////////////////////////////////////////////////////////////////////////////////
// System Settings

#define SYS_FREQ    80000000    // Hz, for CPU
#define MOT_FREQ    1000        // Hz, for PWM
#define ENC_FREQ    400000      // Hz, for encoder signal
#define CTRL_FREQ   550         // Hz, for control loop
    // Standard control loop frequencies are 550Hz and 2200KHz, according to
    // Contelec Vert-x13 user's manual

#define PWM_DIV     2   	// System Clock ticks per PWM Clock tick
#define NUM_DOFS   	12
#define ENC_MIN     0
#define ENC_MAX     16383

#define MOT_PERIOD          SYS_FREQ/PWM_DIV/MOT_FREQ
#define CTRL_PERIOD         SYS_FREQ/CTRL_FREQ
#define ENC_BIT_PERIOD      SYS_FREQ/ENC_FREQ
#define PWM_RANGE           MOT_PERIOD*4/10
#define PWM_ZERO            MOT_PERIOD/2
#define PWM_MAX             MOT_PERIOD*89/100
#define PWM_MIN             MOT_PERIOD*11/100
#define ENC_CNT             (ENC_MAX - ENC_MIN)/2

// Joint 		J00 J01 J02 J03 J04 J05 J06 J07 J08 J09 J10 J11
// Encoder Pin 	PB0	PB1	PB2	PB3	PD2	PD3	PD6	PD7	PE0	PE1	PE2	PE3
// Motor Pin 	PB6	PB7	PB4	PB5	PE4	PE5	PD0	PD1	PA6	PA7	PF2	PF3

#define enc00   (*((volatile uint32_t *)0x40005004))
#define enc01   (*((volatile uint32_t *)0x40005008))
#define enc02   (*((volatile uint32_t *)0x40005010))
#define enc03   (*((volatile uint32_t *)0x40005020))
#define enc04   (*((volatile uint32_t *)0x40007010))
#define enc05   (*((volatile uint32_t *)0x40007020))
#define enc06   (*((volatile uint32_t *)0x40007100))
#define enc07   (*((volatile uint32_t *)0x40007200))
#define enc08   (*((volatile uint32_t *)0x40024004))
#define enc09   (*((volatile uint32_t *)0x40024008))   // old     new
#define enc10   (*((volatile uint32_t *)0x40024010))   // //////  //////
#define enc11   (*((volatile uint32_t *)0x40024020))   // green   brown
#define encSS   (*((volatile uint32_t *)0x40004040))   // yellow  yellow
#define encCl   (*((volatile uint32_t *)0x40004080))   // white   green

// UART Commands
// These constants don't actually need to be in this file, as they are hardware-
// independent, but there is probably no reason to change them on a regular
// basis.

#define CMD_LEN 		10
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
#define ESTOP_RUN       0x0532AC00	// random 32-bit key
#define GAZE_LOCK       0x000000A7  // random 8-bit key
