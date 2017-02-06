// config.h
// This file contains configuration settings for normal operation of Dreamer
// head.

// This file is part of Dreamer Head t0901
// Travis Llado, travis@travisllado.com
// Last modified 2016.09.27

////////////////////////////////////////////////////////////////////////////////
// User Settings

#define GAINS_ACTIVE    1   // debug variable, lets us set all gains to zero

#define D_LENGTH    10  // # cycles
#define I_LENGTH    550 // # cycles (keep in mind, 100 cycles = 4.8KB)

// J00: Lower Neck Pitch
// Encoder Pin: PB0
// Motor Pin:   PB6
#define J00_Min 4425
#define J00_Max 7330
#define J00_Cnt 6500
#define J00_KPn 50  // numerator
#define J00_KPd 1   // denominator
#define J00_KIn 2
#define J00_KId 1
#define J00_KDn 2
#define J00_KDd 1
// J01: Neck Rotation
// Encoder Pin: PB1
// Motor Pin:   PB7
#define J01_Min 7090
#define J01_Max 14670
#define J01_Cnt (J01_Min + J01_Max)/2
#define J01_KPn 20
#define J01_KPd 1
#define J01_KIn 1
#define J01_KId 1
#define J01_KDn 1
#define J01_KDd 3
// J02: Neck Roll
// Encoder Pin: PB2
// Motor Pin:   PB4
#define J02_Min 4035
#define J02_Max 5610
#define J02_Cnt (J02_Min + J02_Max)/2
#define J02_KPn 20
#define J02_KPd 1
#define J02_KIn 1
#define J02_KId 1
#define J02_KDn 1
#define J02_KDd 3
// J03: Upper Neck Pitch
// Encoder Pin: PB3
// Motor Pin:   PB5
#define J03_Min 11210
#define J03_Max 13510
#define J03_Cnt 12475     
#define J03_KPn 20
#define J03_KPd 1
#define J03_KIn 1
#define J03_KId 1
#define J03_KDn 1
#define J03_KDd 1

// J04: Eye Pitch
// Encoder Pin: PD2
// Motor Pin:   PE4
#define J04_Min 10050
#define J04_Max 12360
#define J04_Cnt 11650
#define J04_KPn 5
#define J04_KPd 1
#define J04_KIn 2
#define J04_KId 1
#define J04_KDn 0
#define J04_KDd 1
// J05: Left Eye Yaw
// Encoder Pin: PD3
// Motor Pin:   PE5

#define J05_Min 100
#define J05_Max 3300
#define J05_Cnt 1750
#define J05_KPn 5
#define J05_KPd 1
#define J05_KIn 2
#define J05_KId 1
#define J05_KDn 1
#define J05_KDd 10
// J06: Right Eye Yaw
// Encoder Pin: PD6
// Motor Pin:   PD0
#define J06_Min 8100
#define J06_Max 11240
#define J06_Cnt 9525
#define J06_KPn 5
#define J06_KPd 1
#define J06_KIn 2
#define J06_KId 1
#define J06_KDn 1
#define J06_KDd 10
// J07: Eyelids
// Encoder Pin: PD7
// Motor Pin:   PD1
#define J07_Min 1200
#define J07_Max 13200
#define J07_Cnt 9500
#define J07_KPn 5
#define J07_KPd 1
#define J07_KIn 1
#define J07_KId 10
#define J07_KDn 1
#define J07_KDd 10
// J08: Right Ear Rotation
// Encoder Pin: PE0
// Motor Pin:   PA6
#define J08_Min 640
#define J08_Max 14310
#define J08_Cnt (J08_Min + J08_Max)/2
#define J08_KPn 5
#define J08_KPd 1
#define J08_KIn 1
#define J08_KId 10
#define J08_KDn 1
#define J08_KDd 10
// J09: Right Ear Extension
// Encoder Pin: PE1
// Motor Pin:   PA7
#define J09_Min 1790
#define J09_Max 16220
#define J09_Cnt (J09_Min + J09_Max)/2
#define J09_KPn 5
#define J09_KPd 1
#define J09_KIn 1
#define J09_KId 10
#define J09_KDn 1
#define J09_KDd 10
// J10: Left Ear Rotation
// Encoder Pin: PE2
// Motor Pin:   PF2
#define J10_Min 595
#define J10_Max 14295
#define J10_Cnt (J10_Min + J10_Max)/2
#define J10_KPn 5
#define J10_KPd 1
#define J10_KIn 1
#define J10_KId 10
#define J10_KDn 1
#define J10_KDd 10
// J11: Left Ear Extension
// Encoder Pin: PE3
// Motor Pin:   PF3
#define J11_Min 1765
#define J11_Max 16115
#define J11_Cnt (J11_Min + J11_Max)/2
#define J11_KPn 5
#define J11_KPd 1
#define J11_KIn 1
#define J11_KId 10
#define J11_KDn 1
#define J11_KDd 10

// Ear Light Colors
#define COLOR_BLUE      0x0000001F
#define COLOR_BLUEGREEN 0x00070017
#define COLOR_GREEN     0x001F0000
#define COLOR_MAGENTA   0x00000F0F
#define COLOR_ORANGE    0x00071700
#define COLOR_PINK      0x00051505
#define COLOR_PURPLE    0x00000B14
#define COLOR_RED       0x00001F00
#define COLOR_WHITE     0x000F0F0F
#define COLOR_YELLOW    0x00171700

#define LIGHT_FREQ		60          	// FPS
#define HARD_STOP_COLOR COLOR_BLUEGREEN
#define RUN_COLOR 		COLOR_ORANGE
#define SOFT_STOP_COLOR COLOR_YELLOW

////////////////////////////////////////////////////////////////////////////////
// System Settings

#define SYS_FREQ    80000000    // Hz, for CPU
#define MOT_FREQ    1000        // Hz, for PWM
#define ENC_FREQ    400000      // Hz, for encoder signal
#define CTRL_FREQ   550         // Hz, for control loop
    // Standard control loop frequencies are 550Hz and 2200KHz, according to
    // Contelec Vert-x13 user's manual

#define PWM_DIV     2           // System Clock ticks per PWM Clock tick
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
#define ENC_CNTR            (ENC_MAX - ENC_MIN)/2

#define enc00   (*((volatile uint32_t *)0x40005004))   // PB0 
#define enc01   (*((volatile uint32_t *)0x40005008))   // PB1
#define enc02   (*((volatile uint32_t *)0x40005010))   // PB2
#define enc03   (*((volatile uint32_t *)0x40005020))   // PB3
#define enc04   (*((volatile uint32_t *)0x40007010))   // PD2
#define enc05   (*((volatile uint32_t *)0x40007020))   // PD3
#define enc06   (*((volatile uint32_t *)0x40007100))   // PD6
#define enc07   (*((volatile uint32_t *)0x40007200))   // PD7
#define enc08   (*((volatile uint32_t *)0x40024004))   // PE0
#define enc09   (*((volatile uint32_t *)0x40024008))   // PE1  old     new
#define enc10   (*((volatile uint32_t *)0x40024010))   // PE2  //////  //////
#define enc11   (*((volatile uint32_t *)0x40024020))   // PE3  green   brown
#define encSS   (*((volatile uint32_t *)0x40004040))   // PA4  yellow  yellow
#define encCl   (*((volatile uint32_t *)0x40004080))   // PA5  white   green

// UART Commands
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
#define ESTOP_RUN       0x0532AC00	// random number, acts as a key
