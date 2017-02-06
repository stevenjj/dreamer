// config.h
// This file contains configuration settings for normal operation of Dreamer
// head.

// This file is part of Dreamer Head t0901
// Travis Llado, travis@travisllado.com
// Last modified 2016.12.09

#include "systemConfig.h"

////////////////////////////////////////////////////////////////////////////////
// User Settings

// Debug Settings
#define GAINS_ACTIVE    1   // turns all gains on/off

// PID Settings
#define D_LENGTH        10  // # cycles
#define I_LENGTH        550 // # cycles (keep in mind, 100 cycles = 4.8KB)

// Neck Joints          Eye Joints          Ear Joints
// J00: Neck Pitch      J04: Eye Pitch      J08: Right Ear Rotation
// J01: Head Yaw        J05: Left Eye Yaw   J09: Right Ear Extension
// J02: Head Roll       J06: Right Eye Yaw  J10: Left Ear Rotation
// J03: Head Pitch      J07: Eyelids        J11: Left Ear Rotation

// Can't do the usual 80-character lines here. Forgive me.
// 
//                                  		J00     J01     J02     J03     J04     J05     J06     J07     J08     J09     J10     J11
// Joint Ranges
extern const uint32_t J_MAX[NUM_DOFS] = {  	7330,   14670,  5610,   13510,  12360,  3300,   11240,  13200,  14310,  16220,  14295,  16115   };
extern const uint32_t J_CNT[NUM_DOFS] = {  	6500,   10880,  4823,   12475,  11650,  1750,   9525,   9500,   7475,   9005,   7445,   8940    };
extern const uint32_t J_MIN[NUM_DOFS] = {  	4425,   7090,   4035,   11210,  10050,  100,    8100,   1200,   640,    1790,   595,    1765    };
// PID P Gains (numerator, denominator)
extern const uint32_t KP_N[NUM_DOFS] =  {  	5,      5,      5,      5,      0,      0,      0,      0,      0,      0,      0,      0       };
extern const uint32_t KP_D[NUM_DOFS] =  {  	1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1       };
// PID I Gains
extern const uint32_t KI_N[NUM_DOFS] =  {  	0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0       };
extern const uint32_t KI_D[NUM_DOFS] =  {  	1,      1,      1,      1,      1,      1,      1,      10,     10,     10,     10,     10      };
// PID D Gains
extern const uint32_t KD_N[NUM_DOFS] =  {  	0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0       };
extern const uint32_t KD_D[NUM_DOFS] =  {  	1,      3,      3,      1,      10,     10,     10,     10,     10,     10,     10,     10      };

// Ear Light Colors
#define COLOR_BLUE      0x0000001F
#define COLOR_BLUEGREEN 0x00070017
#define COLOR_GREEN     0x001F0000
#define COLOR_MAGENTA   0x00000F0F
#define COLOR_ORANGE    0x00071700
#define COLOR_PINK      0x00051505
#define COLOR_PURPLE    0x00000B14
#define COLOR_RED       0x00001F00
#define COLOR_WHITE     0x000F0F0F
#define COLOR_YELLOW    0x00171700

#define LIGHT_FREQ      60  // FPS
#define PULSE_INCREMENT 3   // Ears pulse at 255/LIGHT_FREQ/n Hz
#define ERROR_COLOR		COLOR_RED
#define HARD_STOP_COLOR COLOR_BLUEGREEN
#define RUN_COLOR       COLOR_ORANGE
#define SOFT_STOP_COLOR	COLOR_YELLOW
