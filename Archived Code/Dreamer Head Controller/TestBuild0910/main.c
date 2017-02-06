// main.c
// Main program for operation of joints in Dreamer's head. Contains all control
// functions.

// This file is part of Dreamer Head t0910
// Travis Llado, travis@travisllado.com
// Last modified 2016.12.09
 
////////////////////////////////////////////////////////////////////////////////
// Dependencies
 
#include <cstdlib>
#include <stdint.h>
#include "encoders.h"
#include "estop.h"
#include "fastTrig.h"
#include "motors.h"
#include "PLL.h"
#include "systemConfig.h"
#include "tm4c123gh6pm.h"
#include "UART.h"
#include "userConfig.h"

#include <stdlib.h>
#include <tgmath.h>

////////////////////////////////////////////////////////////////////////////////
// Global Variables

int32_t ctrlVar[NUM_DOFS] = {PWM_ZERO};
int32_t ctrlZero[NUM_DOFS] = {PWM_ZERO};
int32_t posActl[NUM_DOFS] = {ENC_CNT};
int32_t posDesr[NUM_DOFS] = {ENC_CNT};
int32_t posCtrl[NUM_DOFS] = {ENC_CNT};
uint32_t timeNow = 0;
int32_t velActl[NUM_DOFS] = {0};
 
////////////////////////////////////////////////////////////////////////////////
// Internal Prototypes

// import from startup.s
void DisableInterrupts(void);
void EnableInterrupts(void);
// import from lights.s
void lightsInit(void);
void lightsUpdate(uint32_t);
// local
void calcVel(void);
void encoderFailsafe(void);
void MJC(void);
void parse(uint32_t command);
void PID(void);
void pulsingLights(uint32_t color);
void randomGaze(void);
void steps(void);
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
    encodersInit(posActl);
    motorInit();
    motorUpdate(ctrlZero); // Initialize motors to motionless
    lightsInit();
    lightsUpdate(ERROR_COLOR);
    UART_Init();
    Timer1_Init();
    // Set desired positions to joint centers
    for(uint32_t i = 0; i < NUM_DOFS; i++)
        posDesr[i] = J_CNT[i];
    // Disengage software e-stop
    softRun();
    
    // Send welcome message to UART terminal
    UART_OutString("Reset.");
 
    // Spin forever
    while(1) {
        parse(UART_InUDec()); // read commands from UART
        
        // All other functions performed by Timer 1 interrupt handler
    }
}

////////////////////////////////////////////////////////////////////////////////
// calcVel()
// Calculates joint absolute velocities.
 
void calcVel(void) {
    static int32_t posPast[NUM_DOFS][D_LENGTH] = {0};
    static uint32_t next = 0;
    
    for(uint32_t i = 0; i < NUM_DOFS; i++) {
        velActl[i] = (posActl[i] - posPast[i][next])*CTRL_FREQ/D_LENGTH;
        posPast[i][next] = posActl[i];
    }

    next = (next + 1)%D_LENGTH;
}
 
////////////////////////////////////////////////////////////////////////////////
// encoderFailsafe()
// Stops motion on any joints where encoder reads out of bounds.
 
void encoderFailsafe(void) {
    for(uint32_t i = 0; i < NUM_DOFS; i++) {
        if(posActl[i] == ENC_MIN || posActl[i] == ENC_MAX)
            ctrlVar[i] = PWM_ZERO;
    }
}

////////////////////////////////////////////////////////////////////////////////
// MJC()
// Modified implementation of Neville Hogan's minimum jerk controller.
// Reformulated to operate at maximum allowed acceleration instead of user-
// specified time.

//uint32_t M_n[NUM_DOFS] = {1};
//uint32_t M_d[NUM_DOFS] = {1};
//uint32_t d = 0;
//int32_t xddd = 0;

//void MJC(void) {
//    const uint32_t n_n = 11547; // magic numerator
//    const uint32_t n_d = 2000;  // magic demoninator
//    
//    for(uint32_t i = 0; i < NUM_DOFS; i++) {
//        // calculate time from current to desired at max allowed acceleration
//        d = sqrt(abs(posDesr[i] - posActl[i])*n_n*M_d[i]/n_d/M_n[i]);
//        // calculate desired current derivatives
//        
//        xddd = -60*posActl[i]/d/d/d - 36*velActl[i]/d/d + 60*posDesr[i]/d/d/d;
//        posCtrl[i] = posActl[i] + xddd/(CTRL_FREQ/10);
//        
//        if(i == 0) {
//            UART_OutUDec(posActl[i]);
//            UART_OutChar(' ');
//            UART_OutUDec(xddd);
//            UART_OutChar(CR);UART_OutChar(LF);
//        }
//    }
//}

//void steps(void) {
//    int32_t n = 10;
//    for(uint32_t i = 0; i < NUM_DOFS; i++) {
//        if((posActl[i] - posDesr[i])*(posActl[i] - posDesr[i]) < n*n*n*n/4)
//            posCtrl[i] = posDesr[i];
//        else if((posActl[i] - posDesr[i]) > velActl[i]*velActl[i]/CTRL_FREQ/CTRL_FREQ)
//            posCtrl[i] = posActl[i] + velActl[i]*n/CTRL_FREQ - n*n/2;
//        else
//            posCtrl[i] = posActl[i] + velActl[i]*n/CTRL_FREQ + n*n/2;
//    }
//    
//    UART_OutUDec(posActl[0]);
//    UART_OutChar(' ');
//    UART_OutUDec(posCtrl[0]);
//    UART_OutChar(CR);UART_OutChar(LF);
//}

////////////////////////////////////////////////////////////////////////////////
// parse()
// Reads incoming UART command and performs appropriate action.

void parse(uint32_t command) {
    uint32_t gazeKey = (command & 0xFF000000) >> 24;
    if(gazeKey == GAZE_LOCK) {
//        uint32_t focusCommand = (command&0x00FF0000)>>16;
//        uint32_t pitchCommand = (command&0x0000FF00)>>8;
//        uint32_t yawCommand = command&0x000000FF;
    }
}

////////////////////////////////////////////////////////////////////////////////
// PID()
// Performs PID control of all twelve joints. Uses 

void PID() {
    // Static variables for P error calculation
    int32_t errP[NUM_DOFS] = {0};
    
    // Static variables for I error calculation
    int32_t errI[NUM_DOFS] = {0};
    static int32_t pastErrI[NUM_DOFS][I_LENGTH] = {0};
    static uint32_t nextI = 0;
    static int32_t errIRaw[NUM_DOFS] = {0};

    // Static variables for D error calculation
    int32_t errD[NUM_DOFS] = {0};
    static int32_t pastErrD[NUM_DOFS][D_LENGTH] = {0};
    static uint32_t nextD = 0;

    for(uint32_t i = 0; i < NUM_DOFS; i++) {
        // Calculate P error
        errP[i] = posActl[i] - posCtrl[i];

        // Calculate I error
        errIRaw[i] -= pastErrI[i][nextI];
        pastErrI[i][nextI] = errP[i];
        errIRaw[i] += pastErrI[i][nextI];
        errI[i] = errIRaw[i]/CTRL_FREQ;

        // Calculate D error
        errD[i] = (errP[i] - pastErrD[i][nextD])*CTRL_FREQ/D_LENGTH;
        pastErrD[i][nextD] = errP[i];

        // Calculate PID control value
        ctrlVar[i] = errP[i]*KP_N[i]/KP_D[i]
                   + errI[i]*KI_N[i]/KI_D[i]
                   + errD[i]*KD_N[i]/KD_D[i]
                   + PWM_ZERO;
    }
    
    nextI = (nextI + 1)%I_LENGTH;
    nextD = (nextD + 1)%D_LENGTH;
}
 
////////////////////////////////////////////////////////////////////////////////
// pulsingLights()
// Varies ear light brightness as a sinusoid.

void pulsingLights(uint32_t color) {

    static uint32_t lightsCounter = 0;
    uint32_t modValue = sin8(lightsCounter);
    uint32_t green = (color & 0x00FF0000) >> 16;
    uint32_t red = (color & 0x0000FF00) >> 8;
    uint32_t blue = (color & 0x000000FF);

    green = green*(modValue + 256)/511;
    red = red*(modValue + 256)/511;
    blue = blue*(modValue + 256)/511;
    modValue = (green << 16) + (red << 8) + blue;

    lightsUpdate(modValue);

    lightsCounter = lightsCounter + PULSE_INCREMENT;
}

////////////////////////////////////////////////////////////////////////////////
// randomGaze()
// Plans motions for random gaze demo

void randomGaze(void) {
    // random gaze goes here
    // this should be the default behavior
    // user can specify other behaviors at any time
}

////////////////////////////////////////////////////////////////////////////////
// Timer1A_Handler()
// Runs function assigned to control frequency.
 
void Timer1A_Handler(void) {
    TIMER1_ICR_R = 0x01;    // acknowledge timer1A timeout

    static uint32_t lightsUpdateCounter = 0;
    uint32_t nextColor = ERROR_COLOR;

    // Check E-Stop statuses right now
    uint32_t eStopNow = eStopStatus();
    
    encodersRead();
    calcVel();
    PID();

    // if e-stops are disengaged ...
    if(eStopNow == RUNNING) {
        randomGaze();
        
        
        encoderFailsafe();
        
        #if GAINS_ACTIVE
            motorUpdate(ctrlVar);
        #else
            motorUpdate(ctrlZero);
        #endif
        
        nextColor = RUN_COLOR;
    }
    // If either e-stop is engaged ...
    else {
        motorUpdate(ctrlZero);  // redundant, motors will be stopped by circuit
        nextColor = SOFT_STOP_COLOR;

        // If hardware e-stop is engaged ...
        if((eStopNow & HARD_STOPPED) >> 1)
            nextColor = HARD_STOP_COLOR;
    }
    
    // Update ear lights
    if(lightsUpdateCounter == 0) {
        pulsingLights(nextColor);
        lightsUpdateCounter = CTRL_FREQ/LIGHT_FREQ;
    }
    lightsUpdateCounter--;

    // Update clock
    timeNow++;
 
    // Debug Output
    UART_OutUDec(posActl[0]);
    UART_OutChar(' ');
    UART_OutUDec(timeNow);
    UART_OutChar(CR);UART_OutChar(LF);
}
 
////////////////////////////////////////////////////////////////////////////////
// Timer1_Init() 
// Initializes all hardware needed to use Timer 1A for control loop

void Timer1_Init(void) {
    DisableInterrupts();
        SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
                                            // activate TIMER1
        while((SYSCTL_PRTIMER_R & SYSCTL_PRTIMER_R1) == 0) {}
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
