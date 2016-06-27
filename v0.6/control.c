// control.c
// Contains all control functions for Dreamer Head.

// This file is part of Dreamer Head v0.5.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.26

////////////////////////////////////////////////////////////////////////////////
// Dependencies

#include <stdint.h>
#include "config.h"
#include "control.h"
#include "tm4c123gh6pm.h"

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

////////////////////////////////////////////////////////////////////////////////
// Internal Prototypes

void calcErrP(void);
void calcErrI(void);
void calcErrD(void);
void parse(uint32_t command);
void PID(void);
void Timer1A_Handler(void);
void Timer1_Init(void);

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

int32_t * PID(void) {
    calcErrP();
    calcErrI();
    calcErrD();

    ctrl[0] = errP[0]*J00_KPn/J00_KPd
            + errI[0]*J00_KIn/J00_KId
            + errD[0]*J00_KPn/J00_KPd
            + PWM_ZERO;
    ctrl[1] = PWM_ZERO
            + errP[1]*J01_KPn/J01_KPd
            + errI[1]*J01_KIn/J01_KId
            + errD[1]*J01_KPn/J01_KPd;
    ctrl[2] = PWM_ZERO
            + errP[2]*J02_KPn/J02_KPd
            + errI[2]*J02_KIn/J02_KId
            + errD[2]*J02_KPn/J02_KPd;
    ctrl[3] = PWM_ZERO
            + errP[3]*J03_KPn/J03_KPd
            + errI[3]*J03_KIn/J03_KId
            + errD[3]*J03_KPn/J03_KPd;
    ctrl[4] = PWM_ZERO
            + errP[4]*J04_KPn/J04_KPd
            + errI[4]*J04_KIn/J04_KId
            + errD[4]*J04_KPn/J04_KPd;
    ctrl[5] = PWM_ZERO
            + errP[5]*J05_KPn/J05_KPd
            + errI[5]*J05_KIn/J05_KId
            + errD[5]*J05_KPn/J05_KPd;
    ctrl[6] = PWM_ZERO
            + errP[6]*J06_KPn/J06_KPd
            + errI[6]*J06_KIn/J06_KId
            + errD[6]*J06_KPn/J06_KPd;
    ctrl[7] = PWM_ZERO
            + errP[7]*J07_KPn/J07_KPd
            + errI[7]*J07_KIn/J07_KId
            + errD[7]*J07_KPn/J07_KPd;
    ctrl[8] = PWM_ZERO
            + errP[8]*J08_KPn/J08_KPd
            + errI[8]*J08_KIn/J08_KId
            + errD[8]*J08_KPn/J08_KPd;
    ctrl[9] = PWM_ZERO
            + errP[9]*J09_KPn/J09_KPd
            + errI[9]*J09_KIn/J09_KId
            + errD[9]*J09_KPn/J09_KPd;
    ctrl[10] = PWM_ZERO
             + errP[10]*J10_KPn/J10_KPd
             + errI[10]*J10_KIn/J10_KId
             + errD[10]*J10_KPn/J10_KPd;
    ctrl[11] = PWM_ZERO
             + errP[11]*J11_KPn/J11_KPd
             + errI[11]*J11_KIn/J11_KId
             + errD[11]*J11_KPn/J11_KPd;

    return ctrl;
}
