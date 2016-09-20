// config.h

////////////////////////////////////////////////////////////////////////////////
// User Settings

// Standard control loop frequencies are 550Hz and 2200KHz, according to
// Contelec Vert-x13 user's manual
#define CTRL_FREQ   550 // Hz
#define LIGHT_FREQ  10  // fraction of CTRL_FREQ, 1/n

#define D_LENGTH    10  // # cycles
#define I_LENGTH    550 // # cycles (keep in mind, 100 cycles = 4.8KB)

// J00: Lower Neck Pitch
// Encoder Pin: PB0
// Motor Pin:   PB6
#define J00_KPn 1   // numerator
#define J00_KPd 10  // denominator
#define J00_KIn 0
#define J00_KId 1
#define J00_KDn 0
#define J00_KDd 1
// J01: Neck Rotation
// Encoder Pin: PB1
// Motor Pin:   PB7
#define J01_KPn 1
#define J01_KPd 1
#define J01_KIn 0
#define J01_KId 1
#define J01_KDn 0
#define J01_KDd 1
// J02: Neck Roll
// Encoder Pin: PB2
// Motor Pin:   PB4
#define J02_KPn 1
#define J02_KPd 1
#define J02_KIn 0
#define J02_KId 1
#define J02_KDn 0
#define J02_KDd 1
// J03: Upper Neck Pitch
// Encoder Pin: PB3
// Motor Pin:   PB5
#define J03_KPn 1
#define J03_KPd 1
#define J03_KIn 0
#define J03_KId 1
#define J03_KDn 0
#define J03_KDd 1

// J04: Eye Pitch
// Encoder Pin: PD2
// Motor Pin:   PE4
#define J04_KPn 1
#define J04_KPd 1
#define J04_KIn 0
#define J04_KId 1
#define J04_KDn 0
#define J04_KDd 1
// J05: Right Eye Yaw
// Encoder Pin: PD3
// Motor Pin:   PE5
#define J05_KPn 1
#define J05_KPd 1
#define J05_KIn 0
#define J05_KId 1
#define J05_KDn 0
#define J05_KDd 1
// J06: Left Eye Yaw
// Encoder Pin: PD6
// Motor Pin:   PD0
#define J06_KPn 1
#define J06_KPd 1
#define J06_KIn 0
#define J06_KId 1
#define J06_KDn 0
#define J06_KDd 1
// J07: Eyelids
// Encoder Pin: PD7
// Motor Pin:   PD1
#define J07_KPn 1
#define J07_KPd 1
#define J07_KIn 0
#define J07_KId 1
#define J07_KDn 0
#define J07_KDd 1

// J08: Right Ear Rotation
// Encoder Pin: PE0
// Motor Pin:   PA6
#define J08_KPn 1
#define J08_KPd 1
#define J08_KIn 0
#define J08_KId 1
#define J08_KDn 0
#define J08_KDd 1
// J09: Right Ear Extension
// Encoder Pin: PE1
// Motor Pin:   PA7
#define J09_KPn 1
#define J09_KPd 1
#define J09_KIn 0
#define J09_KId 1
#define J09_KDn 0
#define J09_KDd 1
// J10: Left Ear Rotation
// Encoder Pin: PE2
// Motor Pin:   PF2
#define J10_KPn 1
#define J10_KPd 1
#define J10_KIn 0
#define J10_KId 1
#define J10_KDn 0
#define J10_KDd 1
// J11: Left Ear Extension
// Encoder Pin: PE3
// Motor Pin:   PF3
#define J11_KPn 1
#define J11_KPd 1
#define J11_KIn 0
#define J11_KId 1
#define J11_KDn 0
#define J11_KDd 1

////////////////////////////////////////////////////////////////////////////////
// System Settings

#define SYS_FREQ            80000000    // Hz
#define MOT_FREQ            10000       // Hz
#define ENC_FREQ            400000      // Hz
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
#define LIGHT_BIT_PERIOD    SYS_FREQ/LIGHT_BIT_FREQ
#define LIGHT_SIGNAL_LENGTH LIGHTS_PER_STRING*BITS_PER_LIGHT
#define PWM_RANGE           MOT_PERIOD*4/10
#define PWM_ZERO            MOT_PERIOD/2
#define PWM_MAX             MOT_PERIOD*9/10
#define PWM_MIN             MOT_PERIOD*1/10
#define ENC_CNTR            (ENC_MAX - ENC_MIN)/2

#define enc00   (*((volatile unsigned long *)0x40005004))   // PB0
#define enc01   (*((volatile unsigned long *)0x40005008))   // PB1
#define enc02   (*((volatile unsigned long *)0x40005010))   // PB2
#define enc03   (*((volatile unsigned long *)0x40005020))   // PB3
#define enc04   (*((volatile unsigned long *)0x40007010))   // PD2
#define enc05   (*((volatile unsigned long *)0x40007020))   // PD3
#define enc06   (*((volatile unsigned long *)0x40007100))   // PD6
#define enc07   (*((volatile unsigned long *)0x40007200))   // PD7
#define enc08   (*((volatile unsigned long *)0x40024004))   // PE0
#define enc09   (*((volatile unsigned long *)0x40024008))   // PE1
#define enc10   (*((volatile unsigned long *)0x40024010))   // PE2
#define enc11   (*((volatile unsigned long *)0x40024020))   // PE3
#define encSS   (*((volatile unsigned long *)0x40004040))   // PA4
#define encCl   (*((volatile unsigned long *)0x40004080))   // PA5
