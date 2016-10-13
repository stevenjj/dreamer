#include <stdint.h>
#include "eightBitSine.h"

uint32_t sineQuarter[65] = {
	128, 131, 134, 137, 140, 144, 147, 150, 153, 156, 159, 162, 165, 168, 171, 
	174, 177, 179, 182, 185, 188, 191, 193, 196, 199, 201, 204, 206, 209, 211, 
	213, 216, 218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 239, 240, 
	241, 243, 244, 245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 
	254, 255, 255, 255, 255
};

uint32_t eightBitSine(uint32_t x) {
	x = x%255;
	
	if(x < 64)
		return sineQuarter[x];
	if(x < 128)
		return sineQuarter[128 - x];
	if(x < 192)
		return 256 - sineQuarter[x - 128];
	return 256 - sineQuarter[256 - x];
}