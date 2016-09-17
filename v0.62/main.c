// main.c
// Main program for operation of joints in Dreamer's head. Contains all controls
// functions.

// This file is part of Dreamer Head v0.4.
// Travis Llado, travis@travisllado.com
// Last modified 2016.09.04

////////////////////////////////////////////////////////////////////////////////
// Dependencies

#include <cstdlib>
#include <stdint.h>
#include "config.h"
#include "cosines.h"
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

int32_t ctrlVar[NUM_DOFS] = {PWM_ZERO};
int32_t ctrlZero[NUM_DOFS] = {PWM_ZERO};
int32_t desrPos[NUM_DOFS] = {
    J00_Cnt,  J01_Cnt,  J02_Cnt,  J03_Cnt,  J04_Cnt,  J05_Cnt,
    J06_Cnt,  J07_Cnt,  J08_Cnt,  J09_Cnt,  J10_Cnt,  J11_Cnt};
int32_t actlPos[NUM_DOFS] = {0};
int32_t errP[NUM_DOFS] = {0};
int32_t errI[NUM_DOFS] = {0};
int32_t errD[NUM_DOFS] = {0};
uint32_t hardStopPos = 0;
uint32_t softStopPos = 0;
uint32_t lightsUpdateCounter = 0;


////////////////////////////////////////////////////////////////////////////////
// Internal Prototypes

void calcErrP(void);
void calcErrI(void);
void calcErrD(void);
void encoderFailsafe(void);
void motionPlan(void);
void parse(uint32_t command);
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
    eStopInit();
    encoderInit(actlPos);
    motorInit();
    lightsInit();
    UART_Init();
    Timer1_Init();
    softRun();

    // Spin forever
    while(1) {
        parse(UART_InUDec()); // read commands from UART

        // All other functions performed by Timer 1 interrupt handler
    }
}

////////////////////////////////////////////////////////////////////////////////
// calcErrP()
// Calculates current position error.

void calcErrP(void) {
    for(uint32_t i = 0; i < NUM_DOFS; i++)
        errP[i] = actlPos[i] - desrPos[i];
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
// encoderFailsafe()
// Stops all motion if any encoders reads out of bounds.

void encoderFailsafe(void) {
    for(uint32_t i = 0; i < NUM_DOFS; i++) {
        if(actlPos[i] == 0 || actlPos[i] == 16383)
            ctrlVar[i] = PWM_ZERO;
    }
}

////////////////////////////////////////////////////////////////////////////////
// motionPlan()
// Moves the head around randomly so it looks less creepy

// tuning parameters; these define motion plan, spacially and temporally
const int32_t focusMax = 300;   // default = 300
const int32_t focusMin = -750;// default = -750
const int32_t yawMax = 3000;    // default = 3000
const int32_t yawMin = -3000;   // default = -3000
const int32_t pitchMax = 500;   // default = 500
const int32_t pitchMin = -1000; // default = -1000
const uint32_t lookPeriodMin = 2;
const uint32_t lookPeriodMax = 6;
const int32_t moveLength_n = 2;
const int32_t moveLength_d = 3;
const int32_t movePeriod = CTRL_FREQ*moveLength_n/moveLength_d;
const uint32_t blinkPeriodMin = 1;
const uint32_t blinkPeriodMax = 10;
const uint32_t blinkLength_n = 1;
const uint32_t blinkLength_d = 3;
const int32_t blinkLength = CTRL_FREQ*blinkLength_n/blinkLength_d;
const int32_t eyelidsMax = J07_Max - 700;
const int32_t eyelidsMin = J07_Min;
const int32_t restartDelay = CTRL_FREQ/2;

// intermediate variables
uint32_t lookCount = 1;
int32_t moveCount = 1;
uint32_t blinkCount1 = CTRL_FREQ*blinkPeriodMin;
int32_t blinkCount2 = blinkLength;
int32_t gazeFocus = 0;
int32_t gazePitch = 0;
int32_t gazeYaw = 0;
int32_t eyelidsDes = 0;
int32_t desrOld[NUM_DOFS] = {
    J00_Cnt,  J01_Cnt,  J02_Cnt,  J03_Cnt,  J04_Cnt,  J05_Cnt,
    J06_Cnt,  J07_Cnt,  J08_Cnt,  J09_Cnt,  J10_Cnt,  J11_Cnt};
int32_t desrNew[NUM_DOFS] = {
    J00_Cnt,  J01_Cnt,  J02_Cnt,  J03_Cnt,  J04_Cnt,  J05_Cnt,
    J06_Cnt,  J07_Cnt,  J08_Cnt,  J09_Cnt,  J10_Cnt,  J11_Cnt};

int32_t change = 0;
int32_t outputNum = 0;

void motionPlan() {
    // if gaze transition is in progress, ...
    if(lookCount == 0) {
        // if gaze transition is just starting, pick new gaze target
        if(moveCount == movePeriod) {
            // store old positions
            for(uint32_t i = 0; i < NUM_DOFS; i++)
                desrOld[i] = desrNew[i];
            
            // calculate new gaze parameters, independent from previous gaze
            gazeFocus = rand()%(focusMax - focusMin) + focusMin;
            gazePitch = rand()%(pitchMax - pitchMin) + pitchMin;
            gazeYaw = rand()%(yawMax - yawMin) + yawMin;

            // convert gaze parameters to joint positions
            desrNew[0] = J00_Cnt + gazeFocus;       // neck pitch
            desrNew[1] = J01_Cnt + gazeYaw*1/2;     // head rotation
            desrNew[2] = J02_Cnt - gazeYaw*1/4;     // head roll
            desrNew[3] = J03_Cnt + gazePitch*2/3 + gazeFocus;
            desrNew[4] = J04_Cnt - gazePitch*1/3;   // eye pitch
            desrNew[5] = J05_Cnt - gazeYaw*1/2;     // left eye yaw
            desrNew[6] = J06_Cnt - gazeYaw*1/2;     // right eye yaw
            desrNew[7] = eyelidsMax + 4*(gazeFocus - focusMax); // eyelids
            desrNew[8] = J08_Cnt + gazePitch*4;     // right ear rotation
            desrNew[9] = J09_Cnt + gazeFocus*4;     // right ear extension
            desrNew[10] = J10_Cnt - gazePitch*4;    // left ear rotation
            desrNew[11] = J11_Cnt + gazeFocus*4;    // left ear extension
            
            moveCount--;
        }
        // if gaze transition is complete, reset counters
        else if(moveCount == 0) {
            // store new values directly as desired values, instead of
            // interpolating
            for(uint32_t i = 0; i < NUM_DOFS; i++)
                desrPos[i] = desrNew[i];

            // pick random time for next gaze transition
            lookCount = rand()%(CTRL_FREQ*(lookPeriodMax - lookPeriodMin)) 
                        + CTRL_FREQ*lookPeriodMin;

            // reset transition counter
            moveCount = movePeriod;
        }
        // if gaze is in progress, interpolate between previous and next
        else {
            uint32_t sinPos = sine1[1000*moveCount/movePeriod];
            for(uint32_t i = 0; i < NUM_DOFS; i++)
                desrPos[i] = desrOld[i] + (desrNew[i] - desrOld[i])*sinPos/1000;

            moveCount--;
        }
    }
    // if gaze transition is not in progress, decrement counter
    else {
        desrPos[7] = desrNew[7];
        lookCount--;
    }

    // Blink Planner
    // if blink is in progress, ...
    if(blinkCount1 == 0) {
        // if blink is completed, reset counters
        if(blinkCount2 == 0) {
            blinkCount1 = rand()%(CTRL_FREQ*(blinkPeriodMax - blinkPeriodMin)) 
                          + CTRL_FREQ*blinkPeriodMin;
            blinkCount2 = blinkLength;
        }
        else {
            // if in first half of blink, interpolate from open to closed
            if(blinkCount2 > blinkLength/2)
                desrPos[7] = eyelidsMin + (desrPos[7] - eyelidsMin)             \
                             *(blinkCount2 - blinkLength/2)/(blinkLength/2);
            // if in second half of blink, interpolate from closed to open
            else
                desrPos[7] = eyelidsMin + (desrPos[7] - eyelidsMin)             \
                             *(blinkLength/2 - blinkCount2)/(blinkLength/2);
        }
        blinkCount2--;
    }
    else
        blinkCount1--;
}

////////////////////////////////////////////////////////////////////////////////
// parse()
// Reads incoming UART command and performs appropriate action.

void parse(uint32_t command) {
    uint32_t address = (command&0xF0000000)>>28;
    uint32_t value = (command&0x0FFFFFFF);

    if(address == MESSAGE_J00_J01) {
        desrPos[0] = (value&0x0FFFC000)>>14;
        desrPos[1] = value&0x00003FFF;
    }
    else if(address == MESSAGE_J02_J03) {
        desrPos[2] = (value&0x0FFFC000)>>14;
        desrPos[3] = value&0x00003FFF;
    }
    else if(address == MESSAGE_J04_J05) {
        desrPos[4] = (value&0x0FFFC000)>>14;
        desrPos[5] = value&0x00003FFF;
    }
    else if(address == MESSAGE_J06_J07) {
        desrPos[6] = (value&0x0FFFC000)>>14;
        desrPos[7] = value&0x00003FFF;
    }
    else if(address == MESSAGE_J08_J09) {
        desrPos[8] = (value&0x0FFFC000)>>14;
        desrPos[9] = value&0x00003FFF;
    }
    else if(address == MESSAGE_J10_J11) {
        desrPos[10] = (value&0x0FFFC000)>>14;
        desrPos[11] = value&0x00003FFF;
    }

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
               + errD[0]*J00_KDn/J00_KDd
               + PWM_ZERO;
    ctrlVar[1] = errP[1]*J01_KPn/J01_KPd
               + errI[1]*J01_KIn/J01_KId
               + errD[1]*J01_KDn/J01_KDd
               + PWM_ZERO;
    ctrlVar[2] = errP[2]*J02_KPn/J02_KPd
               + errI[2]*J02_KIn/J02_KId
               + errD[2]*J02_KDn/J02_KDd
               + PWM_ZERO;
    ctrlVar[3] = errP[3]*J03_KPn/J03_KPd
               + errI[3]*J03_KIn/J03_KId
               + errD[3]*J03_KDn/J03_KDd
               + PWM_ZERO;
    ctrlVar[4] = errP[4]*J04_KPn/J04_KPd
               + errI[4]*J04_KIn/J04_KId
               + errD[4]*J04_KDn/J04_KDd
               + PWM_ZERO;
    ctrlVar[5] = errP[5]*J05_KPn/J05_KPd
               + errI[5]*J05_KIn/J05_KId
               + errD[5]*J05_KDn/J05_KDd
               + PWM_ZERO;
    ctrlVar[6] = errP[6]*J06_KPn/J06_KPd
               + errI[6]*J06_KIn/J06_KId
               + errD[6]*J06_KDn/J06_KDd
               + PWM_ZERO;
    ctrlVar[7] = errP[7]*J07_KPn/J07_KPd
               + errI[7]*J07_KIn/J07_KId
               + errD[7]*J07_KDn/J07_KDd
               + PWM_ZERO;
    ctrlVar[8] = errP[8]*J08_KPn/J08_KPd
               + errI[8]*J08_KIn/J08_KId
               + errD[8]*J08_KDn/J08_KDd
               + PWM_ZERO;
    ctrlVar[9] = errP[9]*J09_KPn/J09_KPd
               + errI[9]*J09_KIn/J09_KId
               + errD[9]*J09_KDn/J09_KDd
               + PWM_ZERO;
    ctrlVar[10]= errP[10]*J10_KPn/J10_KPd
               + errI[10]*J10_KIn/J10_KId
               + errD[10]*J10_KDn/J10_KDd
               + PWM_ZERO;
    ctrlVar[11]= errP[11]*J11_KPn/J11_KPd
               + errI[11]*J11_KIn/J11_KId
               + errD[11]*J11_KDn/J11_KDd
               + PWM_ZERO;
}

////////////////////////////////////////////////////////////////////////////////
// Timer1A_Handler()
// Runs function assigned to control frequency.

void Timer1A_Handler(void) {
    TIMER1_ICR_R = 0x01;    // acknowledge timer1A timeout

    encoderRead();
    PID();
    encoderFailsafe();
    motorUpdate(ctrlVar);
    int32_t eStopStatus = STOPPED;
    int32_t nextColor = 0;

    if(hardStopStatus() == STOPPED)
        nextColor = HARD_STOP_COLOR;
    else if(softStopStatus() == STOPPED)
        nextColor = SOFT_STOP_COLOR;
    else {
        nextColor = RUN_COLOR;
        eStopStatus = RUNNING;
    }

    lightsUpdateCounter++;
    if(lightsUpdateCounter == LIGHT_UPDATE_PERIOD) {
        lightsUpdate(nextColor);
        lightsUpdateCounter = 0;
    }

    if(eStopStatus == RUNNING)
        motionPlan();
    // if EStop is stopped, ...
    else {
        // store current (unactuated) position as desired for controller and 
        // planner
        for(uint32_t i = 0; i < NUM_DOFS; i++) {
            desrNew[i] = actlPos[i];
            desrPos[0] = desrNew[0];
        }

        // When EStop is re-started, set timer to hold current position, then 
        // perform gaze transition after short delay
        lookCount = restartDelay;
        moveCount = movePeriod;
        blinkCount1 = 1;
    }

    UART_OutUDec(actlPos[2]);
    UART_OutChar(' ');
    UART_OutUDec(actlPos[3]);
    UART_OutChar(CR);UART_OutChar(LF);
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
