// lights.h
// Contains all code required to control two set of ear lights.

// This file is part of Dreamer Head v0.4.
// Travis Llado, travis@travisllado.com
// Last modified 2016.06.22

////////////////////////////////////////////////////////////////////////////////
// Global Variables

#define COLOR_RED       0x00001F00
#define COLOR_ORANGE    0x000F1F00
#define COLOR_YELLOW    0x001F1F00
#define COLOR_GREEN     0x001F0000
#define COLOR_BLUE      0x0000001F
#define COLOR_PURPLE    0x00001F1F
#define COLOR_WHITE     0x001F1F1F
#define COLOR_PINK      0x000F1F0F

////////////////////////////////////////////////////////////////////////////////
// lightsInit()
// Performs all hardware setup for chandelier light string cluster. Takes input
// of function to be performed each frame.

void lightsInit(void);

////////////////////////////////////////////////////////////////////////////////
// updateLights()
// Performs actions to commence writing of current contents of LED[] to LEDs.
  
//void lightsUpdate(uint32_t newColor);
