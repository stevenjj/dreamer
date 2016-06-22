// control.c
// Contains all functions for control systems for Dreamer's head.

// This file is part of Dreamer Head v0.1.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.18

////////////////////////////////////////////////////////////////////////////////
// Dependencies

#include <stdint.h>
#include "config.h"
#include "control.h"
#include "encoders.h"
#include "tm4c123gh6pm.h"

void DisableInterrupts(void);
void EnableInterrupts(void);
long StartCritical (void);
void EndCritical(long sr);

// void (*loopTask)(void);

////////////////////////////////////////////////////////////////////////////////
// Internal Prototypes

void controlInit(void);// (*newFunc)(void), uint32_t *inArray, uint32_t *outArray);
void Timer1A_Handler(void);
void Timer1_Init(void);

////////////////////////////////////////////////////////////////////////////////
// controlInit()

void controlInit(void) {// (*newFunc)(void), uint32_t *inArray, uint32_t *outArray) {
    // loopTask = newFunc;
    // posn = inArray;
    // ctrl = outArray;
    
    Timer1_Init();
}

////////////////////////////////////////////////////////////////////////////////
// Timer1A_Handler()
// Runs function assigned to control frequency.

void Timer1A_Handler(void) {
    TIMER1_ICR_R = 0x01;    // acknowledge timer1A timeout
encoderRead();
    // loopTask();
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
        TIMER1_CTL_R = 0x01;                // enable TIMER1A
    EnableInterrupts();
}
