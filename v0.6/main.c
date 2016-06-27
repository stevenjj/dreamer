// main.c
// Main program for operation of joints in Dreamer's head. Contains all controls
// functions.

// This file is part of Dreamer Head v0.5.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.26

////////////////////////////////////////////////////////////////////////////////
// Dependencies

#include <stdint.h>
#include "config.h"
#include "controls.h"
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

uint32_t desrColor = runningColor;
int32_t desrPos[NUM_DOFS] = {ENC_CNTR};

const int32_t ctrlZero[NUM_DOFS] = {PWM_ZERO};
const uint32_t runningColor = 0x00003F00;
const uint32_t softStoppedColor = 0x0000003F;
const uint32_t hardStoppedColor = 0x003F0000;

////////////////////////////////////////////////////////////////////////////////
// Internal Prototypes

void parse(uint32_t command);
void Timer1A_Handler(void);
void Timer1_Init(void);

////////////////////////////////////////////////////////////////////////////////
// int main()
// Main program for operation of Dreamer's Head. Initializes hardware required
// for reading of encoders and control of motors, then runs an infinite loop as
// all further actions are performed by interrupt handlers.

int main(void) {
    // Initialize system clock
    PLL_Init();
    // Initialize robot hardware
    eStopInit();
    encoderInit(actlPos);
    controlsInit(actlPos, desrPos, ctrlVar);
    lightsInit();
    UART_Init();
    motorInit();
    Timer1_Init();

    // Spin forever
    while(1) {
        parse(UART_InUDec()); // read commands from UART
    }
}

////////////////////////////////////////////////////////////////////////////////
// parse()
// Reads incoming UART command and performs appropriate action.

void parse(uint32_t message) {
    uint32_t address = (message&0xF0000000)>>28;
    uint32_t value = (message&0x0FFFFFFF);

    if(address == MESSAGE_ESTOP) {
        if(value == ESTOP_RUN)
            eStopSoftRun();
        else
            eStopSoftStop();
    }
    else if(address == MESSAGE_J00_J01) {
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
    else if(address == MESSAGE_LIGHTS) {
        desrColor = value;
    }

}

////////////////////////////////////////////////////////////////////////////////
// Timer1A_Handler()
// Runs function assigned to control frequency.

void Timer1A_Handler(void) {
    TIMER1_ICR_R = 0x01;    // acknowledge timer1A timeout
    
    if(eStopHardRunning()) {
        motorUpdate(PID(desrPos, encoderRead()));
//        UART_OutUDec(posn[0]);UART_OutChar(' ');UART_OutString("running");UART_OutChar(CR);UART_OutChar(LF);
    }
    else {
        motorUpdate(ctrlZero);
//        UART_OutUDec(posn[0]);UART_OutChar(' ');UART_OutString("stopped");UART_OutChar(CR);UART_OutChar(LF);
    }

    lightsUpdate(desrColor);
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
