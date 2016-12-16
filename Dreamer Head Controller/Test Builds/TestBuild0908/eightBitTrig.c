// eightBitTrig.c
// Contains all values and code for eightBitSine/...Cosine/...Tangent

// This file is part of Dreamer Head t0907
// Travis Llado, travis@travisllado.com
// Last modified 2016.10.12

////////////////////////////////////////////////////////////////////////////////
// Dependencies

#include <stdint.h>
#include "eightBitTrig.h"

////////////////////////////////////////////////////////////////////////////////
// Global Variables

uint32_t sineQuarter[65] = {
    128, 131, 134, 137, 140, 144, 147, 150, 153, 156, 159, 162, 165, 168, 171, 
    174, 177, 179, 182, 185, 188, 191, 193, 196, 199, 201, 204, 206, 209, 211, 
    213, 216, 218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 239, 240, 
    241, 243, 244, 245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 
    254, 255, 255, 255, 255
};

uint32_t tangentHalf[65] = {
    128, 128, 128, 128, 128, 128, 128, 129, 129, 129, 129, 129, 129, 129, 129, 
    129, 129, 129, 129, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 131, 
    131, 131, 131, 131, 131, 131, 132, 132, 132, 132, 132, 133, 133, 133, 134, 
    134, 134, 135, 135, 136, 136, 137, 138, 139, 140, 141, 143, 145, 148, 152, 
    158, 169, 189, 250, 128
};

////////////////////////////////////////////////////////////////////////////////
// eightBitSine()
// Produces low resolution sine values as quickly as possible. Takes input of
// uint32_t x and returns uint32_t 127*sine(2πx/256)+128. In other words,
// produces an integer-resolution sine wave at 128±127 (instead of 0±1) and
// period 255 (instead of 2π).

uint32_t eightBitSine(uint32_t x) {
    x = x%256;
    
    if(x < 64)
        return sineQuarter[x];
    if(x < 128)
        return sineQuarter[128 - x];
    if(x < 192)
        return 256 - sineQuarter[x - 128];
    return 256 - sineQuarter[256 - x];
}

////////////////////////////////////////////////////////////////////////////////
// eightBitCosine()
// Produces low resolution cosine values as quickly as possible. Properties are
// identical to those of eightBitSine.

uint32_t eightBitCosine(uint32_t x) {
    x = x%256;
    
    if(x < 64)
        return sineQuarter[64 - x];
    if(x < 128)
        return 256 - sineQuarter[x - 64];
    if(x < 192)
        return 256 - sineQuarter[192 - x];
    return sineQuarter[x - 192];
}

////////////////////////////////////////////////////////////////////////////////
// eightBitTangent()
// Produces low resolution tangent values as quickly as possible. Takes input of
// uint32_t x and returns uint32_t 3*tangent(2πx/256)+128. In other words,
// produces an integer-resolution tangent plot centered around 128 and 3x normal
// scale (because 1x scale would be useless at integer resolution) and period
// 255 (instead of 2π).

uint32_t eightBitTangent(uint32_t x) {
    x = x%128;
    
    if(x < 64)
        return tangentHalf[x];
    return 256 - tangentHalf[128 - x];
}
