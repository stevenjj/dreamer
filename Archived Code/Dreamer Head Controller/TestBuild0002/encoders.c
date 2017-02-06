// encoders.c
// Contains all functions needed to initialize and read twelve Contelec Vert-X
// 13-E encoders.

// This file is part of Dreamer Head v0.1.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.18

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

uint32_t tickCount = 0;

const uint32_t clkTick[MESSAGE_LENGTH] = {
    // start
    0x00000000, 
    // write byte 1 (20µs)
    0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 
    0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 
    0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 
    0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 
    // pause (15µs)
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    // write byte 2 (20µs)
    0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 
    0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 
    0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 
    0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 
    // pause (15µs)
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    // read byte 3 (20µs)
    0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 
    0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 
    0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 
    0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 
    // pause (15µs)
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    // read byte 4 (20µs)
    0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 
    0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 
    0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 
    0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000
};
const uint32_t datTick[MESSAGE_LENGTH] = {
    // start (1 tick)
    0x00000000, 
    // write byte 1 (20µs)
    0x0000FFFF, 0x0000FFFF, 0x00000000, 0x00000000, 
    0x0000FFFF, 0x0000FFFF, 0x00000000, 0x00000000, 
    0x0000FFFF, 0x0000FFFF, 0x00000000, 0x00000000, 
    0x0000FFFF, 0x0000FFFF, 0x00000000, 0x00000000, 
    // pause (15µs)
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    // write byte 2 (20µs)
    0x0000FFFF, 0x0000FFFF, 0x00000000, 0x00000000, 
    0x0000FFFF, 0x0000FFFF, 0x00000000, 0x00000000, 
    0x0000FFFF, 0x0000FFFF, 0x00000000, 0x00000000, 
    0x0000FFFF, 0x0000FFFF, 0x00000000, 0x00000000, 
    // pause (15µs)
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    // read byte 3 (20µs)
    0x0000000F, 0x0000000F, 0x0000000E, 0x0000000E, 
    0x0000000D, 0x0000000D, 0x0000000C, 0x0000000C, 
    0x0000000B, 0x0000000B, 0x0000000A, 0x0000000A, 
    0x00000009, 0x00000009, 0x00000080, 0x00000080, 
    // pause (15µs)
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    // read byte 4 (20µs)
    0x00000007, 0x00000007, 0x00000006, 0x00000006, 
    0x00000005, 0x00000005, 0x00000004, 0x00000004, 
    0x00000003, 0x00000003, 0x00000002, 0x00000002, 
    0x00000001, 0x00000001, 0x00000000, 0x00000000
};

////////////////////////////////////////////////////////////////////////////////
// Internal Prototypes

void PortA_Init(void);
void PortB_Init(void);
void PortD_Init(void);
void PortE_Init(void);
void Timer1_Init(void);
void Timer2_Init(void);

////////////////////////////////////////////////////////////////////////////////
// encoderInit()
// Initializes all GPIO pins needed to read twelve joint encoders.

void encoderInit(void) {
    PortA_Init();
    PortB_Init();
    PortD_Init();
    PortE_Init();
    Timer1_Init();
    Timer2_Init();
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

////////////////////////////////////////////////////////////////////////////////
// Starts sequence of visualization and LED update for a single frame.

void Timer1A_Handler(void) {
    TIMER1_ICR_R = 0x01;    // acknowledge timer1A timeout

    tickCount = 0;          // reset message counter
    encSS = 0x0000;         // start message
    TIMER2_CTL_R = 0x01;    // enable Timer 2A
}

////////////////////////////////////////////////////////////////////////////////
// Timer1_Init()
// Initializes all hardware needed to use Timer 1A for frame update

void Timer1_Init(void) {
    DisableInterrupts();
        SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
                                            // activate TIMER1
        while((SYSCTL_PRTIMER_R&SYSCTL_PRTIMER_R1) == 0) {}
                                            // wait for timer to start
        TIMER1_CTL_R = 0x00;                // disable TIMER1A during setup
        TIMER1_CFG_R = 0x00;                // set to 32-bit mode
        TIMER1_TAMR_R = 0x02;               // set to periodic mode
        TIMER1_TAILR_R = CTRL_PERIOD - 1;  // set reset value
        TIMER1_TAPR_R = 0;                  // set bus clock resolution
        TIMER1_ICR_R = 0x01;                // clear TIMER1A timeout flag
        TIMER1_IMR_R |= 0x01;               // arm timeout interrupt
        NVIC_PRI5_R = (NVIC_PRI5_R & 0x00FFFFFF) | (1 << 29);
                                            // set priority
        NVIC_EN0_R = 1 << 21;               // enable IRQ 21 in NVIC
        TIMER1_CTL_R = 0x01;    // enable Timer 1A
    EnableInterrupts();
}

////////////////////////////////////////////////////////////////////////////////
// Timer2A_Handler()
// Writes single bit to all LED strings

void Timer2A_Handler(void) {

    TIMER2_ICR_R = 0x01;    // acknowledge timer2A timeout
    
    if(tickCount < 46) {
        encCl = clkTick[tickCount];
        enc00 = datTick[tickCount];
        enc01 = datTick[tickCount];
        enc02 = datTick[tickCount];
        enc03 = datTick[tickCount];
        enc04 = datTick[tickCount];
        enc05 = datTick[tickCount];
        enc06 = datTick[tickCount];
        enc07 = datTick[tickCount];
        enc08 = datTick[tickCount];
        enc09 = datTick[tickCount];
        enc10 = datTick[tickCount];
        enc11 = datTick[tickCount];
    }
    else if(tickCount == 46) {
        // change all pins to input
        GPIO_PORTA_DIR_R &= ~0x0C;
        GPIO_PORTB_DIR_R &= ~0x0F;
        GPIO_PORTD_DIR_R &= ~0xCC;
        GPIO_PORTE_DIR_R &= ~0x0F;
    }
    else if(tickCount < MESSAGE_LENGTH) {
        encCl = clkTick[tickCount];
        // newBit00 = enc00;
        // 11 more
    }
    else {
        TIMER2_CTL_R = 0x00;    // disable Timer 2A
        encSS = 0xFFFF;         // end message
        
        // change DAT lines to output
        GPIO_PORTA_DIR_R |= 0x0C;
        GPIO_PORTB_DIR_R |= 0x0F;
        GPIO_PORTD_DIR_R |= 0xCC;
        GPIO_PORTE_DIR_R |= 0x0F;

        encCl = 0x0000;
        enc00 = 0x0000;
        // 11 more
    }

    tickCount++;
}

////////////////////////////////////////////////////////////////////////////////
// Timer2_Init()
// Initializes all hardware needed to use Timer2A for bit updates

void Timer2_Init(void) {
    DisableInterrupts();
        SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
                                            // activate timer2
        while((SYSCTL_PRTIMER_R&SYSCTL_PRTIMER_R2) == 0) {}
                                            // wait for timer to start
        TIMER2_CTL_R = 0x00;                // disable TIMER2A during setup
        TIMER2_CFG_R = 0x00;                // set to 32-bit mode
        TIMER2_TAMR_R = 0x02;               // set to periodic mode
        TIMER2_TAILR_R = ENC_PERIOD/2 - 1;  // set reset value
        TIMER2_TAPR_R = 0;                  // set bus clock resolution
        TIMER2_ICR_R = 0x01;                // clear TIMER1A timeout flag
        TIMER2_IMR_R |= 0x01;               // arm timeout interrupt
        NVIC_PRI5_R = (NVIC_PRI5_R & 0x00FFFFFF) | (1 << 29);
                                            // set priority
        NVIC_EN0_R = 1 << 23;               // enable interrupt 23 in NVIC
    EnableInterrupts();
}
