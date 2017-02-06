// config.h

////////////////////////////////////////////////////////////////////////////////
// User Settings

#define CTRL_FREQ   550         // Hz
#define I_LENGTH    550         // # control cycles

////////////////////////////////////////////////////////////////////////////////
// System Settings

#define SYS_FREQ    80000000    // Hz
#define MOT_FREQ    10000       // Hz
#define ENC_FREQ    400000      // Hz
#define ENC_MIN     0
#define ENC_MAX     4182
#define PWM_DIV     2           // System Clock ticks per PWM Clock tick
#define NUM_DOFS    12

#define MOT_PERIOD  SYS_FREQ/PWM_DIV/MOT_FREQ
#define CTRL_PERIOD SYS_FREQ/CTRL_FREQ
#define BIT_PERIOD  SYS_FREQ/ENC_FREQ
#define PWM_RANGE   MOT_PERIOD*4/10
#define PWM_ZERO    MOT_PERIOD/2
#define PWM_MAX     MOT_PERIOD*9/10
#define PWM_MIN     MOT_PERIOD*1/10

#define encSS   (*((volatile unsigned long *)0x40004040))   // PA4
#define encCl   (*((volatile unsigned long *)0x40004080))   // PA5
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
