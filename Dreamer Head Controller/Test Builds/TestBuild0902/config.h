// config.h
// This file contains configuration settings for normal operation of Dreamer
// head.

// This file is part of Dreamer Head t0901
// Travis Llado, travis@travisllado.com
// Last modified 2016.09.27

////////////////////////////////////////////////////////////////////////////////
// User Settings

#define D_LENGTH    10  // # cycles
#define I_LENGTH    550 // # cycles (keep in mind, 100 cycles = 4.8KB)

// J00: Lower Neck Pitch
// Encoder Pin: PB0
// Motor Pin:   PB6
#define J00_Min 4490
#define J00_Max 7325
#define J00_Cnt 6500
#define J00_KPn 50   // numerator
#define J00_KPd 1   // denominator
#define J00_KIn 2
#define J00_KId 1
#define J00_KDn 2
#define J00_KDd 1
// J01: Neck Rotation
// Encoder Pin: PB1
// Motor Pin:   PB7
#define J01_Min 7100
#define J01_Max 14675
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
#define J02_Min 430
#define J02_Max 1965
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
#define J03_Max 13505
#define J03_Cnt 12400     
#define J03_KPn 20
#define J03_KPd 1
#define J03_KIn 1
#define J03_KId 1
#define J03_KDn 1
#define J03_KDd 1

// J04: Eye Pitch
// Encoder Pin: PD2
// Motor Pin:   PE4
#define J04_Min 10035
#define J04_Max 12350
#define J04_Cnt 11300
#define J04_KPn 5
#define J04_KPd 1
#define J04_KIn 2
#define J04_KId 1
#define J04_KDn 0
#define J04_KDd 1
// J05: Left Eye Yaw
// Encoder Pin: PD3
// Motor Pin:   PE5

#define J05_Min 105
#define J05_Max 3305
#define J05_Cnt 1790
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
#define J06_Max 11305
#define J06_Cnt 9625
#define J06_KPn 5
#define J06_KPd 1
#define J06_KIn 2
#define J06_KId 1
#define J06_KDn 1
#define J06_KDd 10
// J07: Eyelids
// Encoder Pin: PD7
// Motor Pin:   PD1
#define J07_Min 3400
#define J07_Max 10880
#define J07_Cnt 9000
#define J07_KPn 10
#define J07_KPd 1
#define J07_KIn 5
#define J07_KId 1
#define J07_KDn 1
#define J07_KDd 5
// J08: Right Ear Rotation
// Encoder Pin: PE0
// Motor Pin:   PA6
#define J08_Min 7210
#define J08_Max 14405
#define J08_Cnt (J08_Min + J08_Max)/2
#define J08_KPn 1
#define J08_KPd 1
#define J08_KIn 0
#define J08_KId 1
#define J08_KDn 0
#define J08_KDd 1
// J09: Right Ear Extension
// Encoder Pin: PE1
// Motor Pin:   PA7
#define J09_Min 1800
#define J09_Max 15485
#define J09_Cnt (J09_Min + J09_Max)/2
#define J09_KPn 1
#define J09_KPd 1
#define J09_KIn 0
#define J09_KId 1
#define J09_KDn 0
#define J09_KDd 1
// J10: Left Ear Rotation
// Encoder Pin: PE2
// Motor Pin:   PF2
#define J10_Min 1980
#define J10_Max 9210
#define J10_Cnt (J10_Min + J10_Max)/2
#define J10_KPn 1
#define J10_KPd 1
#define J10_KIn 0
#define J10_KId 1
#define J10_KDn 0
#define J10_KDd 1
// J11: Left Ear Extension
// Encoder Pin: PE3
// Motor Pin:   PF3
#define J11_Min 1740
#define J11_Max 15310
#define J11_Cnt (J11_Min + J11_Max)/2
#define J11_KPn 1
#define J11_KPd 1
#define J11_KIn 0
#define J11_KId 1
#define J11_KDn 0
#define J11_KDd 1

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
#define ESTOP_RUN       0x0532AC00

// Ear Light Colors

#define COLOR_RED       0x00001F00
#define COLOR_ORANGE    0x000F1F00
#define COLOR_YELLOW    0x001F1F00
#define COLOR_GREEN     0x001F0000
#define COLOR_BLUE      0x000F001F
#define COLOR_PURPLE    0x00001F1F
#define COLOR_WHITE     0x001F1F1F
#define COLOR_PINK      0x000F1F0F

#define HARD_STOP_COLOR COLOR_BLUE
#define RUN_COLOR 		COLOR_ORANGE
#define SOFT_STOP_COLOR COLOR_YELLOW

////////////////////////////////////////////////////////////////////////////////
// System Settings

#define SYS_FREQ            80000000    // Hz
#define MOT_FREQ            1000        // Hz
#define ENC_FREQ            400000      // Hz
#define CTRL_FREQ           550         // Hz
    // Standard control loop frequencies are 550Hz and 2200KHz, according to
    // Contelec Vert-x13 user's manual
#define PWM_DIV             2           // System Clock ticks per PWM Clock tick
#define NUM_DOFS            12
#define ENC_MIN             0
#define ENC_MAX             16383
#define LIGHT_BIT_FREQ      800000      // Hz
#define BITS_PER_LIGHT      24
#define LIGHTS_PER_STRING   40
#define LIGHT_BIT_PRIORITY  1

#define MOT_PERIOD          SYS_FREQ/PWM_DIV/MOT_FREQ
#define CTRL_PERIOD         SYS_FREQ/CTRL_FREQ
#define ENC_BIT_PERIOD      SYS_FREQ/ENC_FREQ
#define LIGHT_UPDATE_PERIOD CTRL_FREQ/10                      // 10 FPS
#define LIGHT_BIT_PERIOD    SYS_FREQ/LIGHT_BIT_FREQ
#define LIGHT_SIGNAL_LENGTH LIGHTS_PER_STRING*BITS_PER_LIGHT
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
