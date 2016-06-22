// main.c
// Main program for operation of joints in Dreamer's head. Contains all controls
// functions.

// This file is part of Dreamer Head v0.4.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.21

////////////////////////////////////////////////////////////////////////////////
// Dependencies

#include <stdint.h>
#include "config.h"
#include "encoders.h"
#include "estop.h"
#include "lights.h"
#include "motors.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"
#include "UART.h"

void DisableInterrupts(void);
void EnableInterrupts(void);
long StartCritical (void);
void EndCritical(long sr);
void WaitForInterrupt(void);

////////////////////////////////////////////////////////////////////////////////
// Global Variables

int32_t desr[NUM_DOFS] = {ENC_CNTR};
int32_t posn[NUM_DOFS] = {0};
int32_t errP[NUM_DOFS] = {0};
int32_t errI[NUM_DOFS] = {0};
int32_t errD[NUM_DOFS] = {0};
int32_t ctrlVar[NUM_DOFS] = {PWM_ZERO};
int32_t ctrlZero[NUM_DOFS] = {PWM_ZERO};
uint32_t runningColor = 0x00003F00;
uint32_t softStoppedColor = 0x0000003F;
uint32_t hardStoppedColor = 0x003F0000;

////////////////////////////////////////////////////////////////////////////////
// Internal Prototypes

void calcErrP(void);
void calcErrI(void);
void calcErrD(void);
void PID(void);
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
    eStopInit();
    lightsInit();
    motorInit();
    Timer1_Init();
    UART_Init();

    // Spin forever
    while(1) {
        // Nothing here! All further actions performed by interrupt handlers.
    }
}

////////////////////////////////////////////////////////////////////////////////
// calcErrP()
// Calculates current position error.

void calcErrP(void) {
    for(uint32_t i = 0; i < NUM_DOFS; i++)
        errP[i] = posn[i] - desr[i];
}

////////////////////////////////////////////////////////////////////////////////
// calcErrI()
// Calculates integral error.

void calcErrI(void) {
    static int32_t pastErrP[NUM_DOFS][I_LENGTH] = {0};
    static uint32_t next = 0;
		static int32_t errIRaw[NUM_DOFS] = {0};

    for(uint32_t i = 0; i < NUM_DOFS; i++) {
        errIRaw[i] -= pastErrP[i][next];
        errIRaw[i] += errP[i];
        errI[i] = errIRaw[i]/CTRL_FREQ;
        pastErrP[i][next] = errP[i];
    }
    next = (next + 1)%I_LENGTH;
}

////////////////////////////////////////////////////////////////////////////////
// calcErrD()
// Calculates derivative error.

void calcErrD(void) {
    static int32_t pastErrP[NUM_DOFS][D_LENGTH] = {0};
    static uint32_t next = 0;

    for(uint32_t i = 0; i < NUM_DOFS; i++) {
        errD[i] = (errP[i] - pastErrP[i][next])*CTRL_FREQ/D_LENGTH;
        pastErrP[i][next] = errP[i];
    }
    next = (next + 1)%D_LENGTH;
}

////////////////////////////////////////////////////////////////////////////////
// PID()
// PID controller for all twelve DOFs.

void PID(void) {
    calcErrP();
    calcErrI();
    calcErrD();

    ctrlVar[0] = errP[0]*J00_KPn/J00_KPd
               + errI[0]*J00_KIn/J00_KId
               + errD[0]*J00_KPn/J00_KPd
               + PWM_ZERO;
    ctrlVar[1] = PWM_ZERO
               + errP[1]*J01_KPn/J01_KPd
               + errI[1]*J01_KIn/J01_KId
               + errD[1]*J01_KPn/J01_KPd;
    ctrlVar[2] = PWM_ZERO
               + errP[2]*J02_KPn/J02_KPd
               + errI[2]*J02_KIn/J02_KId
               + errD[2]*J02_KPn/J02_KPd;
    ctrlVar[3] = PWM_ZERO
               + errP[3]*J03_KPn/J03_KPd
               + errI[3]*J03_KIn/J03_KId
               + errD[3]*J03_KPn/J03_KPd;
    ctrlVar[4] = PWM_ZERO
               + errP[4]*J04_KPn/J04_KPd
               + errI[4]*J04_KIn/J04_KId
               + errD[4]*J04_KPn/J04_KPd;
    ctrlVar[5] = PWM_ZERO
               + errP[5]*J05_KPn/J05_KPd
               + errI[5]*J05_KIn/J05_KId
               + errD[5]*J05_KPn/J05_KPd;
    ctrlVar[6] = PWM_ZERO
               + errP[6]*J06_KPn/J06_KPd
               + errI[6]*J06_KIn/J06_KId
               + errD[6]*J06_KPn/J06_KPd;
    ctrlVar[7] = PWM_ZERO
               + errP[7]*J07_KPn/J07_KPd
               + errI[7]*J07_KIn/J07_KId
               + errD[7]*J07_KPn/J07_KPd;
    ctrlVar[8] = PWM_ZERO
               + errP[8]*J08_KPn/J08_KPd
               + errI[8]*J08_KIn/J08_KId
               + errD[8]*J08_KPn/J08_KPd;
    ctrlVar[9] = PWM_ZERO
               + errP[9]*J09_KPn/J09_KPd
               + errI[9]*J09_KIn/J09_KId
               + errD[9]*J09_KPn/J09_KPd;
    ctrlVar[10] = PWM_ZERO
                + errP[10]*J10_KPn/J10_KPd
                + errI[10]*J10_KIn/J10_KId
                + errD[10]*J10_KPn/J10_KPd;
    ctrlVar[11] = PWM_ZERO
                + errP[11]*J11_KPn/J11_KPd
                + errI[11]*J11_KIn/J11_KId
                + errD[11]*J11_KPn/J11_KPd;
}

////////////////////////////////////////////////////////////////////////////////
// Timer1A_Handler()
// Runs function assigned to control frequency.

void Timer1A_Handler(void) {
    TIMER1_ICR_R = 0x01;    // acknowledge timer1A timeout

    encoderRead();
    
    if(eStopHardRunning()) {
        PID();
        motorUpdate(ctrlVar);
        lightsUpdate(runningColor);
//        UART_OutUDec(posn[0]);UART_OutChar(' ');UART_OutString("running");UART_OutChar(CR);UART_OutChar(LF);
    }
    else {
        motorUpdate(ctrlZero);
        lightsUpdate(hardStoppedColor);
//        UART_OutUDec(posn[0]);UART_OutChar(' ');UART_OutString("stopped");UART_OutChar(CR);UART_OutChar(LF);
    }
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
