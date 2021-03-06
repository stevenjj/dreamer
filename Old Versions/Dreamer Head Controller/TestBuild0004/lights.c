// lights.c
// Contains all code required to control two set of ear lights.

// This file is part of Dreamer Head v0.4.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.22

////////////////////////////////////////////////////////////////////////////////
// Dependencies

#include <stdint.h>
#include "config.h"
#include "lights.h"
#include "tm4c123gh6pm.h"

void DisableInterrupts(void);
void EnableInterrupts(void);
long StartCritical (void);
void EndCritical(long sr);
void WaitForInterrupt(void);

////////////////////////////////////////////////////////////////////////////////
// Global Variables

uint32_t bitCount = 0;
uint32_t lightCount = 0;
uint32_t newColor = 0;

#define PC4 (*((volatile unsigned long *)0x40006040))
#define PC5 (*((volatile unsigned long *)0x40006080))

////////////////////////////////////////////////////////////////////////////////
// Internal Prototypes

void PortC_Init(void);
void Timer3_Init(void);
void Timer3A_Handler(void);

////////////////////////////////////////////////////////////////////////////////
// lightsInit()
// Performs all hardware setup for chandelier light string cluster. Takes input
// of function to be performed each frame.

void lightsInit(void) {
    PortC_Init();
    Timer3_Init();

    lightsUpdate(0x001F1F1F);
}

////////////////////////////////////////////////////////////////////////////////
// updateLights()
// Performs actions to commence writing of current contents of LED[] to LEDs.

void lightsUpdate(uint32_t color) {
    lightCount++;

    if(lightCount == LIGHT_FREQ) {
        newColor = color;
        lightCount = 0;
        bitCount = LIGHT_SIGNAL_LENGTH - 1;   // reset message bit counter
        TIMER3_CTL_R = 0x01;            // enable Timer 3A
    }
}

////////////////////////////////////////////////////////////////////////////////
// PortC_Init()
// Initializes PC4/5 as GPIO output.

void PortC_Init(void) {
    DisableInterrupts();
        SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
                                            // activate port C
        while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R2) == 0) {}
                                            // wait for clock to stabilize
        GPIO_PORTC_DIR_R |= 0x30;           // make PC4/5 output
        GPIO_PORTC_AFSEL_R &= ~0x30;        // disable alt funct on PC4/5
        GPIO_PORTC_DEN_R |= 0x30;           // enable digital I/O on PC4/5
        GPIO_PORTC_PCTL_R &= ~0x00FF0000;   // configure PC4/5 as GPIO
        GPIO_PORTC_AMSEL_R &= ~0x30;        // disable analog funcs on PC4/5
    EnableInterrupts();
}

////////////////////////////////////////////////////////////////////////////////
// Timer3_Init()
// Initializes all hardware needed to use Timer3A for bit updates

void Timer3_Init(void) {
    DisableInterrupts();
        SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3;
                                            // activate timer3
        while((SYSCTL_PRTIMER_R&SYSCTL_PRTIMER_R3) == 0) {}
                                            // wait for timer to start
        TIMER3_CTL_R = 0x00;                // disable TIMER3A during setup
        TIMER3_CFG_R = 0x00;                // set to 32-bit mode
        TIMER3_TAMR_R = 0x02;               // set to periodic mode
        TIMER3_TAILR_R = LIGHT_BIT_PERIOD - 1;    // set reset value
        TIMER3_TAPR_R = 0;                  // set bus clock resolution
        TIMER3_ICR_R = 0x01;                // clear TIMER3A timeout flag
        TIMER3_IMR_R |= 0x01;               // arm timeout interrupt
        NVIC_PRI5_R = (NVIC_PRI5_R & 0x00FFFFFF) | (LIGHT_BIT_PRIORITY << 29);
                                            // set priority
        NVIC_EN1_R = 1 << 3;               // enable interrupt 35 in NVIC
					TIMER3_CTL_R = 0x01;
    EnableInterrupts();
}

////////////////////////////////////////////////////////////////////////////////
// Timer2A_Handler()
// Writes single bit to all LED strings

void Timer3A_Handler(void) {
    volatile uint32_t delay = 0;
    uint32_t bitNum = bitCount%BITS_PER_LIGHT;
    uint32_t nextBit = (newColor&(1<<bitNum))>>(bitNum - 4);

    TIMER3_ICR_R = 0x01;    // acknowledge timer3A timeout
    
    PC4 = 0xFF;         // write PC4 high
    PC5 = 0xFF;         // write PC5 high
    delay=0;delay=0;delay=0;delay=0;delay=0;delay=0;delay=0;
                        // delay remaining 0 time
    PC4 = nextBit;      // write PC4 to message bit
    PC5 = nextBit<<1;   // write PC5 to message bit
    delay=0;delay=0;delay=0;delay=0;delay=0;delay=0;delay=0;delay=0;delay=0;
    delay=0;delay=0;delay=0;delay=0;delay=0;delay=0;delay=0;delay=0;delay=0;
    delay=0;            // delay remaining 1 time
    PC4 = 0x00;         // write PC4 low
    PC5 = 0x00;         // write PC5 low

    bitCount--;
    if(bitCount == 0)           // after entire message is sent
        TIMER3_CTL_R = 0x00;    // disable Timer 3A
}
