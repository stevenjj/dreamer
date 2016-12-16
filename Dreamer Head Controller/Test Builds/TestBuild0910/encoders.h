// encoders.h
// Contains all functions needed to initialize and read twelve Contelec Vert-X
// 13-E encoders.

// This file is part of Dreamer Head v0.1.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.18

////////////////////////////////////////////////////////////////////////////////
// encoderInit()
// Initializes all GPIO pins needed to read twelve joint encoders.

void encodersInit(int32_t *newArray);
void encodersRead(void);
