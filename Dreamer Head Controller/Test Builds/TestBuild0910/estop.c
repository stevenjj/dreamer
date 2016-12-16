// estop.c
// Contains all code required to operate the emergency stop button.

// This file is part of Dreamer Head v0.4.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.22

////////////////////////////////////////////////////////////////////////////////
// Dependencies

#include <stdint.h>
#include "estop.h"
#include "systemConfig.h"
#include "tm4c123gh6pm.h"

void DisableInterrupts(void);
void EnableInterrupts(void);

////////////////////////////////////////////////////////////////////////////////
// Global Variables

#define PC6 (*((volatile unsigned long *)0x40006100))   // senses button posn
#define PC7 (*((volatile unsigned long *)0x40006200))   // controls relay
uint32_t softStopState = 0;

////////////////////////////////////////////////////////////////////////////////
// Internal Prototypes

void GPIOPortC_Handler(void);
void PortC67_Init(void);

////////////////////////////////////////////////////////////////////////////////
// eStopInit()

void eStopInit(void) {
    PortC67_Init();
    softStop();
}

////////////////////////////////////////////////////////////////////////////////
// eStopStatus()

uint32_t eStopStatus(void) {
    return ((GPIO_PORTC_DATA_R & 0x40) >> 5) | softStopState;
}

////////////////////////////////////////////////////////////////////////////////
// softRun()

void softRun(void) {
    PC7 = 0x00;
    softStopState = 0;
}

////////////////////////////////////////////////////////////////////////////////
// softStop()

void softStop(void) {
    PC7 = 0xFF;
    softStopState = 1;
}

////////////////////////////////////////////////////////////////////////////////
// PortC67_Init()
// Initializes PC6/7 as GPIO output.

void PortC67_Init(void) {
    DisableInterrupts();
        SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
                                            // activate port C
        while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R2) == 0) {}
                                            // wait for clock to stabilize
        GPIO_PORTC_DIR_R   &= ~0x40;        // make PC6 input
        GPIO_PORTC_DIR_R   |=  0x80;        // make PC7 output
        GPIO_PORTC_AFSEL_R &= ~0xC0;        // disable alt funct on PC6/7
        GPIO_PORTC_DEN_R   |=  0xC0;        // enable digital I/O on PC6/7
        GPIO_PORTC_PCTL_R  &= ~0xFF000000;  // configure PC6/7 as GPIO
        GPIO_PORTC_AMSEL_R &= ~0xC0;        // disable analog funcs on PC6/7
    EnableInterrupts();
}
