// lights.h
// Contains all code required to control two set of ear lights.

// This file is part of Dreamer Head v0.4.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.22

////////////////////////////////////////////////////////////////////////////////
// lightsInit()
// Performs all hardware setup for chandelier light string cluster. Takes input
// of function to be performed each frame.

void lightsInit(void);

////////////////////////////////////////////////////////////////////////////////
// updateLights()
// Performs actions to commence writing of current contents of LED[] to LEDs.

void lightsUpdate(uint32_t color);
