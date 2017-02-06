// motors.c
// Contains all code required to initialize and update PWM signals used to
// control twelve motors.

// This file is part of Dreamer Head v0.1.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.18

////////////////////////////////////////////////////////////////////////////////
// Dependencies

#include <stdint.h>
#include "motors.h"
#include "systemConfig.h"
#include "tm4c123gh6pm.h"

void DisableInterrupts(void);
void EnableInterrupts(void);
long StartCritical (void);
void EndCritical(long sr);
void WaitForInterrupt(void);

////////////////////////////////////////////////////////////////////////////////
// Global Variables

int32_t speedInit[12] = {PWM_ZERO};

////////////////////////////////////////////////////////////////////////////////
// motorInit()
// Initializes all PWM control signals for 12 motors on pins
// PA6, PA7, PB4, PB5, PB6, PB7, PD0, PD1, PE4, PE5, PF2, PF3.
// Sets PWM frequency to user specified value.

void motorInit(void) {
    // Configure GPIO Pins for PWM Output
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;  // activate clock for PWM Mod 0
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;  // activate clock for PWM Mod 1

    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;    // activate clock for Port A
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;    // activate clock for Port B
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;    // activate clock for Port D
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;    // activate clock for Port E
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;    // activate clock for Port F

    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R0) == 0) {}   // wait for clock
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R1) == 0) {}   // wait for clock
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R3) == 0) {}   // wait for clock
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0) {}   // wait for clock
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0) {}   // wait for clock

    GPIO_PORTA_AFSEL_R |= 0xC0;         // enable alt funct on PA6-7
    GPIO_PORTA_PCTL_R &= ~0xFF000000;   // configure PA6-7 as PWMmod1
    GPIO_PORTA_PCTL_R |= 0x55000000;
    GPIO_PORTA_AMSEL_R &= ~0xC0;        // disable analog function on PA6-7
    GPIO_PORTA_DEN_R |= 0xC0;           // enable digital I/O on PA6-7

    GPIO_PORTB_AFSEL_R |= 0xF0;         // enable alt funct on PB4-7
    GPIO_PORTB_PCTL_R &= ~0xFFFF0000;   // configure PB4-7 as PWMmod0
    GPIO_PORTB_PCTL_R |= 0x44440000;
    GPIO_PORTB_AMSEL_R &= ~0xF0;        // disable analog function on PB4-7
    GPIO_PORTB_DEN_R |= 0xF0;           // enable digital I/O on PB4-7

    GPIO_PORTD_AFSEL_R |= 0x03;         // enable alt funct on PD0-1
    GPIO_PORTD_PCTL_R &= ~0x000000FF;   // configure PD0-1 as PWMmod0
    GPIO_PORTD_PCTL_R |= 0x00000044;
    GPIO_PORTD_AMSEL_R &= ~0x03;        // disable analog function on PD0-1
    GPIO_PORTD_DEN_R |= 0x03;           // enable digital I/O on PD0-1

    GPIO_PORTE_AFSEL_R |= 0x30;         // enable alt funct on PE4-5
    GPIO_PORTE_PCTL_R &= ~0x00FF0000;   // configure PE4-5 as PWMmod0
    GPIO_PORTE_PCTL_R |= 0x00440000;
    GPIO_PORTE_AMSEL_R &= ~0x30;        // disable analog function on PE4-5
    GPIO_PORTE_DEN_R |= 0x30;           // enable digital I/O on PE4-5

    GPIO_PORTF_AFSEL_R |= 0x0C;         // enable alt funct on PF2-3
    GPIO_PORTF_PCTL_R &= ~0x0000FF00;   // configure PF2-3 as PWMmod1
    GPIO_PORTF_PCTL_R |= 0x00005500;
    GPIO_PORTF_AMSEL_R &= ~0x0C;        // disable analog function on PF2-3
    GPIO_PORTF_DEN_R |= 0x0C;           // enable digital I/O on PF2-3
   
    // Set PWM clock to desired fraction of system clock
    SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;
    SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M;
    SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;

    // Configure PWM timing
    PWM0_0_CTL_R = 0;   // set countdown mode for mod 0 block 0
    PWM0_1_CTL_R = 0;   // set countdown mode for mod 0 block 1
    PWM0_2_CTL_R = 0;   // set countdown mode for mod 0 block 2
    PWM0_3_CTL_R = 0;   // set countdown mode for mod 0 block 3
    PWM1_1_CTL_R = 0;   // set countdown mode for mod 1 block 1
    PWM1_3_CTL_R = 0;   // set countdown mode for mod 1 block 3
    PWM0_0_GENA_R = (PWM_0_GENA_ACTCMPAD_ONE|PWM_0_GENA_ACTLOAD_ZERO);
    PWM0_0_GENB_R = (PWM_0_GENB_ACTCMPBD_ONE|PWM_0_GENB_ACTLOAD_ZERO);
    PWM0_1_GENA_R = (PWM_1_GENA_ACTCMPAD_ONE|PWM_1_GENA_ACTLOAD_ZERO);
    PWM0_1_GENB_R = (PWM_1_GENB_ACTCMPBD_ONE|PWM_1_GENB_ACTLOAD_ZERO);
    PWM0_2_GENA_R = (PWM_2_GENA_ACTCMPAD_ONE|PWM_2_GENA_ACTLOAD_ZERO);
    PWM0_2_GENB_R = (PWM_2_GENB_ACTCMPBD_ONE|PWM_2_GENB_ACTLOAD_ZERO);
    PWM0_3_GENA_R = (PWM_3_GENA_ACTCMPAD_ONE|PWM_3_GENA_ACTLOAD_ZERO);
    PWM0_3_GENB_R = (PWM_3_GENB_ACTCMPBD_ONE|PWM_3_GENB_ACTLOAD_ZERO);
    PWM1_1_GENA_R = (PWM_1_GENA_ACTCMPAD_ONE|PWM_1_GENA_ACTLOAD_ZERO);
    PWM1_1_GENB_R = (PWM_1_GENB_ACTCMPBD_ONE|PWM_1_GENB_ACTLOAD_ZERO);
    PWM1_3_GENA_R = (PWM_3_GENA_ACTCMPAD_ONE|PWM_3_GENA_ACTLOAD_ZERO);
    PWM1_3_GENB_R = (PWM_3_GENB_ACTCMPBD_ONE|PWM_3_GENB_ACTLOAD_ZERO);
                                    // define signal triggers
    PWM0_0_LOAD_R = MOT_PERIOD;    // set counter reset values (period)
    PWM0_1_LOAD_R = MOT_PERIOD;
    PWM0_2_LOAD_R = MOT_PERIOD;
    PWM0_3_LOAD_R = MOT_PERIOD;
    PWM1_1_LOAD_R = MOT_PERIOD;
    PWM1_3_LOAD_R = MOT_PERIOD;

    motorUpdate(speedInit); // init comparator values (duty)

    PWM0_0_CTL_R |= PWM_0_CTL_ENABLE;   // start Mod 0 Blk 0
    PWM0_1_CTL_R |= PWM_1_CTL_ENABLE;   // start Mod 0 Blk 1
    PWM0_2_CTL_R |= PWM_2_CTL_ENABLE;   // start Mod 0 Blk 2
    PWM0_3_CTL_R |= PWM_3_CTL_ENABLE;   // start Mod 0 Blk 3
    PWM1_1_CTL_R |= PWM_1_CTL_ENABLE;   // start Mod 1 Blk 1
    PWM1_3_CTL_R |= PWM_3_CTL_ENABLE;   // start Mod 1 Blk 3

    PWM0_ENABLE_R |= (PWM_ENABLE_PWM0EN|PWM_ENABLE_PWM1EN|PWM_ENABLE_PWM2EN|   \
        PWM_ENABLE_PWM3EN|PWM_ENABLE_PWM4EN|PWM_ENABLE_PWM5EN|PWM_ENABLE_PWM6EN\
        |PWM_ENABLE_PWM7EN);    // enable PWM Module 0
    PWM1_ENABLE_R |= (PWM_ENABLE_PWM2EN|PWM_ENABLE_PWM3EN|PWM_ENABLE_PWM6EN    \
        |PWM_ENABLE_PWM7EN);    // enable PWM Module 1
}

////////////////////////////////////////////////////////////////////////////////
// motorUpdate()
// Updates PWM duty cycles corresponding to desired motor speeds for all twelve
// motors.

void motorUpdate(int32_t *speed) {
    // basic bounds check
    for(uint32_t i = 0; i < NUM_DOFS; i++) {
        if(speed[i] > PWM_MAX)
            speed[i] = PWM_MAX;
        else if(speed[i] < PWM_MIN)
            speed[i] = PWM_MIN;
    }
    
    PWM0_0_CMPA_R = speed[0];
    PWM0_0_CMPB_R = speed[1];
    PWM0_1_CMPA_R = speed[2];
    PWM0_1_CMPB_R = speed[3];
    PWM0_2_CMPA_R = speed[5];
    PWM0_2_CMPB_R = speed[7];
    PWM0_3_CMPA_R = speed[4];
    PWM0_3_CMPB_R = speed[6];
    PWM1_1_CMPA_R = speed[10];
    PWM1_1_CMPB_R = speed[11];
    PWM1_3_CMPA_R = speed[9];
    PWM1_3_CMPB_R = speed[8];
}
