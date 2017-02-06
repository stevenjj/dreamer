// fastTrig.c
// Contains all code required for use of eight- and sixteen-bit sine, cosine,
// and tangent functions.

// Travis Llado, travis@travisllado.com
// Last modified 2016.11.13

////////////////////////////////////////////////////////////////////////////////
// Dependencies

#include <stdint.h>
#include "fastTrig.h"

////////////////////////////////////////////////////////////////////////////////
// Global Variables

uint32_t sineQuarter8[65] = {
    128, 131, 134, 137, 140, 144, 147, 150, 153, 156, 159, 162, 165, 168, 171, 
    174, 177, 179, 182, 185, 188, 191, 193, 196, 199, 201, 204, 206, 209, 211, 
    213, 216, 218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 239, 240, 
    241, 243, 244, 245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 
    254, 255, 255, 255, 255
};

//uint32_t tangentHalf8[65] = {
//    128, 128, 128, 128, 128, 128, 128, 129, 129, 129, 129, 129, 129, 129, 129, 
//    129, 129, 129, 129, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 131, 
//    131, 131, 131, 131, 131, 131, 132, 132, 132, 132, 132, 133, 133, 133, 134, 
//    134, 134, 135, 135, 136, 136, 137, 138, 139, 140, 141, 143, 145, 148, 152, 
//    158, 169, 189, 250, 128
//};

//uint32_t sineQuarter16[65] = {
//    32768, 33572, 34376, 35178, 35980, 36779, 37576, 38370, 39161, 39947, 40730,
//    41507, 42280, 43046, 43807, 44561, 45307, 46047, 46778, 47500, 48214, 48919,
//    49614, 50298, 50972, 51636, 52287, 52927, 53555, 54171, 54773, 55362, 55938,
//    56499, 57047, 57579, 58097, 58600, 59087, 59558, 60013, 60451, 60873, 61278,
//    61666, 62036, 62389, 62724, 63041, 63339, 63620, 63881, 64124, 64348, 64553,
//    64739, 64905, 65053, 65180, 65289, 65377, 65446, 65496, 65525, 65535
//};

//uint32_t tangentHalf16[65] = {
//    32768, 32788, 32807, 32827, 32847, 32867, 32887, 32907, 32927, 32947, 32968,
//    32989, 33010, 33032, 33054, 33076, 33099, 33122, 33146, 33170, 33195, 33221,
//    33247, 33274, 33302, 33331, 33361, 33392, 33424, 33457, 33492, 33529, 33567,
//    33607, 33650, 33694, 33742, 33792, 33845, 33902, 33964, 34030, 34101, 34178,
//    34263, 34355, 34457, 34571, 34697, 34839, 35001, 35187, 35402, 35655, 35958,
//    36326, 36785, 37373, 38154, 39246, 40880, 43600, 49032, 65316, 32768
//};

////////////////////////////////////////////////////////////////////////////////
// sin8()
// Produces low resolution sine values as quickly as possible. Takes input of
// uint32_t x and returns uint32_t 127*sine(2px/256)+128. In other words,
// produces an integer-resolution sine wave with amplitude 128±127 (instead of
// 0±1) and period 256 (instead of 2p).

uint32_t sin8(uint32_t x) {
    x = x%256;
    
    if(x < 64)
        return sineQuarter8[x];
    if(x < 128)
        return sineQuarter8[128 - x];
    if(x < 192)
        return 256 - sineQuarter8[x - 128];
    return 256 - sineQuarter8[256 - x];
}

////////////////////////////////////////////////////////////////////////////////
// cos8()
// Produces low resolution cosine values as quickly as possible. Properties are
// identical to those of eightBitSine.

uint32_t cos8(uint32_t x) {
    x = x%256;
    
    if(x < 64)
        return sineQuarter8[64 - x];
    if(x < 128)
        return 256 - sineQuarter8[x - 64];
    if(x < 192)
        return 256 - sineQuarter8[192 - x];
    return sineQuarter8[x - 192];
}

////////////////////////////////////////////////////////////////////////////////
// tan8()
// Produces low resolution tangent values as quickly as possible. Takes input of
// uint32_t x and returns uint32_t 3*tangent(2px/256)+128. In other words,
// produces an integer-resolution tangent plot centered around 128 and 3x normal
// scale (because 1x scale would be useless at integer resolution) and period
// 256 (instead of 2p).

//uint32_t tan8(uint32_t x) {
//    x = x%128;
//    
//    if(x < 64)
//        return tangentHalf8[x];
//    return 256 - tangentHalf8[128 - x];
//}

////////////////////////////////////////////////////////////////////////////////
// sin16()
// Produces low resolution sine values as quickly as possible. Takes input of
// uint32_t x and returns an approximation of 32767*sine(2px/65536)+32768.
// In other words, produces an integer-resolution sine wave with amplitude
// 32768±32767 (instead of 0±1) and period 65536 (instead of 2p).

//uint32_t sin16(uint32_t x) {
//    x = x%65536;
//    uint32_t quotient = x/256;
//    uint32_t remainder = x%256;
//    uint32_t a = 0;
//    uint32_t b = 0;
//    
//    if(quotient < 64) {
//        a = sineQuarter16[quotient];
//        b = sineQuarter16[quotient + 1];
//        return a + (b - a)*remainder/256;
//    }
//    if(quotient < 128) {
//        a = sineQuarter16[128 - quotient];
//        b = sineQuarter16[127 - quotient];
//        return a - (a - b)*remainder/256;
//    }
//    if(quotient < 192) {
//        a = 65536 - sineQuarter16[quotient - 128];
//        b = 65536 - sineQuarter16[quotient - 127];
//        return a - (a - b)*remainder/256;
//    }
//    a = 65536 - sineQuarter16[256 - quotient];
//    b = 65536 - sineQuarter16[255 - quotient];
//    return a + (b - a)*remainder/256;
//}

////////////////////////////////////////////////////////////////////////////////
// cos16()
// Produces low resolution cosine values as quickly as possible. Properties are
// identical to those of sixteenBitSine.

//uint32_t cos16(uint32_t x) {
//    x = x%65536;
//    uint32_t quotient = x/256;
//    uint32_t remainder = x%256;
//    uint32_t a = 0;
//    uint32_t b = 0;
//    
//    if(quotient < 64) {
//        a = sineQuarter16[64 - quotient];
//        b = sineQuarter16[63 - quotient];
//        return a - (a - b)*remainder/256;
//    }
//    if(quotient < 128) {
//        a = 65536 - sineQuarter16[quotient - 64];
//        b = 65536 - sineQuarter16[quotient - 63];
//        return a - (a - b)*remainder/256;
//    }
//    if(quotient < 192) {
//        a = 65536 - sineQuarter16[192 - quotient];
//        b = 65536 - sineQuarter16[191 - quotient];
//        return a + (b - a)*remainder/256;
//    }
//    a = sineQuarter16[quotient - 192];
//    b = sineQuarter16[quotient - 191];
//    return a + (b - a)*remainder/256;
//}

////////////////////////////////////////////////////////////////////////////////
// tan16()
// Produces low resolution tangent values as quickly as possible. Takes input of
// uint32_t x and returns uint32_t 799*tangent(2px/65536)+32768. In other words,
// produces an integer-resolution tangent plot centered around 32768 and 799x
// normal scale (because 1x scale would be useless at integer resolution) and
// period 65536 (instead of 2p).

//uint32_t tan16(uint32_t x) {
//    x = x%32768;
//    uint32_t quotient = x/256;
//    uint32_t remainder = x%256;
//    uint32_t a = 0;
//    uint32_t b = 0;
//    
//    if(quotient < 63) {
//        a = tangentHalf16[quotient];
//        b = tangentHalf16[quotient + 1];
//        return a + (b - a)*remainder/256;
//    }
//    if(quotient == 63)
//        return 65535;
//    if(quotient == 64) {
//    	if(remainder == 0)
//    		return 32768;
//    	return 1;
//    }
//    a = 65536 - tangentHalf16[128 - quotient];
//    b = 65536 - tangentHalf16[127 - quotient];
//    return a + (b - a)*remainder/256;
//}
