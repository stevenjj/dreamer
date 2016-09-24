// motors.h
// Contains all code required to initialize and update PWM signals used to
// control twelve motors.

// This file is part of Dreamer Head v0.1.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.18

////////////////////////////////////////////////////////////////////////////////
// motorInit()
// Initializes all PWM control signals for 12 motors on pins
// PA6, PA7, PB4, PB5, PB6, PB7, PD0, PD1, PE4, PE5, PF2, PF3.
// Sets PWM frequency to user specified value.

void motorInit(void);

////////////////////////////////////////////////////////////////////////////////
// motorUpdate()
// Updates PWM duty cycles corresponding to desired motor speeds for all twelve
// motors.

void motorUpdate(int32_t *speed);
