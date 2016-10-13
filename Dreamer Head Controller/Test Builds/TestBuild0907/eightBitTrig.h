// eightBitTrig.h
// Contains all values and code for eightBitSine/Cosine/Tangent.

// This file is part of Dreamer Head t0907
// Travis Llado, travis@travisllado.com
// Last modified 2016.10.12

////////////////////////////////////////////////////////////////////////////////
// eightBitSine()
// Produces low resolution sine values as quickly as possible. Takes input of
// uint32_t x and returns uint32_t 127*sine(2πx/256)+128. In other words,
// produces an integer-resolution sine wave at 128±127 (instead of 0±1) and
// period 255 (instead of 2π).

uint32_t eightBitSine(uint32_t x);

////////////////////////////////////////////////////////////////////////////////
// eightBitCosine()
// Produces low resolution cosine values as quickly as possible. Properties are
// identical to those of eightBitSine.

uint32_t eightBitCosine(uint32_t x);

////////////////////////////////////////////////////////////////////////////////
// eightBitTangent()
// Produces low resolution tangent values as quickly as possible. Takes input of
// uint32_t x and returns uint32_t 3*tangent(2πx/256)+128. In other words,
// produces an integer-resolution tangent plot centered around 128 and 3x normal
// scale (because 1x scale would be useless at integer resolution) and period
// 255 (instead of 2π).

uint32_t eightBitTangent(uint32_t x);
