// main.c
// Main program for operation of joints in Dreamer's head.

// This file is part of Dreamer Head v0.1.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.18

////////////////////////////////////////////////////////////////////////////////
// Dependencies

#include <stdint.h>
#include "config.h"
#include "control.h"
#include "encoders.h"
#include "motors.h"
#include "PLL.h"

////////////////////////////////////////////////////////////////////////////////
// int main()
// Main program for operation of Dreamer's Head. Initializes hardware required
// for reading of encoders and control of motors, then runs an infinite loop as
// all further actions are performed by interrupt handlers.

int main(void) {
    PLL_Init();
    controlInit();
    encoderInit();
    motorInit();


    uint32_t newSpeed = 0;
    uint32_t speeds[12] = {0};

    while(1) {
        newSpeed += 1;
        newSpeed = newSpeed % 3600;
        if(newSpeed == 0)
            newSpeed = 400;
        for(uint32_t i = 0; i < 12; i++)
            speeds[i] = newSpeed;
				for(uint32_t i = 0; i < 10000; i++)
            newSpeed = newSpeed;
        
        motorUpdate(speeds);
    }
}
