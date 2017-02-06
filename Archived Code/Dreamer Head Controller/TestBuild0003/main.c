// main.c
// Main program for operation of joints in Dreamer's head.

// This file is part of Dreamer Head v0.3.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.19

////////////////////////////////////////////////////////////////////////////////
// Dependencies

#include <stdint.h>
#include "config.h"
#include "encoders.h"
#include "motors.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"

void DisableInterrupts(void);
void EnableInterrupts(void);
long StartCritical (void);
void EndCritical(long sr);
void WaitForInterrupt(void);

////////////////////////////////////////////////////////////////////////////////
// Global Variables

uint32_t posn[NUM_DOFS] = {0};
uint32_t ctrlVar[NUM_DOFS] = {PWM_ZERO};

////////////////////////////////////////////////////////////////////////////////
// Internal Prototypes

void Timer1A_Handler(void);
void Timer1_Init(void);

////////////////////////////////////////////////////////////////////////////////
// int main()
// Main program for operation of Dreamer's Head. Initializes hardware required
// for reading of encoders and control of motors, then runs an infinite loop as
// all further actions are performed by interrupt handlers.

int main(void) {
    // Initialize all hardware
    PLL_Init();
    encoderInit(posn);
    motorInit();
    Timer1_Init();

    // Spin forever
    while(1) {
        // Nothing here! All further actions performed by interrupt handlers.
    }
}

////////////////////////////////////////////////////////////////////////////////
// Timer1A_Handler()
// Runs function assigned to control frequency.

void Timer1A_Handler(void) {
    TIMER1_ICR_R = 0x01;    // acknowledge timer1A timeout

    static uint32_t newSpeed = PWM_MIN;
    static int32_t change = 1;
    if(newSpeed == (PWM_ZERO + PWM_RANGE/4))
        change = -1;
		else if(newSpeed == (PWM_ZERO - PWM_RANGE/4))
        change = 1;
    for(uint32_t i = 0; i < NUM_DOFS; i++)
        ctrlVar[i] = newSpeed;
    newSpeed += change;

    encoderRead();
    motorUpdate(ctrlVar);
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
        TIMER1_TAILR_R = CTRL_PERIOD - 1;   // set reset value
        TIMER1_TAPR_R = 0;                  // set bus clock resolution
        TIMER1_ICR_R = 0x01;                // clear TIMER1A timeout flag
        TIMER1_IMR_R |= 0x01;               // arm timeout interrupt
        NVIC_PRI5_R = (NVIC_PRI5_R & 0x00FFFFFF) | (1 << 29);
                                            // set priority
        NVIC_EN0_R = 1 << 21;               // enable IRQ 21 in NVIC
        TIMER1_CTL_R = 0x01;                // enable TIMER1A
    EnableInterrupts();
}
