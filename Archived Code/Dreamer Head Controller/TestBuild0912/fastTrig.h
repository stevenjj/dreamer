// fastTrig.h
// Contains all code required for use of eight- and sixteen-bit sine, cosine,
// and tangent functions.

// Travis Llado, travis@travisllado.com
// Last modified 2016.11.13

////////////////////////////////////////////////////////////////////////////////
// sin8()
// Produces low resolution sine values as quickly as possible. Takes input of
// uint32_t x and returns uint32_t 127*sine(2px/256)+128. In other words,
// produces an integer-resolution sine wave with amplitude 128±127 (instead of
// 0±1) and period 256 (instead of 2p).

uint32_t sin8(uint32_t x);

////////////////////////////////////////////////////////////////////////////////
// cos8()
// Produces low resolution cosine values as quickly as possible. Properties are
// identical to those of sin8.

uint32_t cos8(uint32_t x);

////////////////////////////////////////////////////////////////////////////////
// tan8()
// Produces low resolution tangent values as quickly as possible. Takes input of
// uint32_t x and returns uint32_t 3*tangent(2px/256)+128. In other words,
// produces an integer-resolution tangent plot centered around 128 and 3x normal
// scale (because 1x scale would be useless at integer resolution) and period
// 256 (instead of 2p).

//uint32_t tan8(uint32_t x);

////////////////////////////////////////////////////////////////////////////////
// sin16()
// Produces low resolution sine values as quickly as possible. Takes input of
// uint32_t x and returns an approximation of 32767*sine(2px/65536)+32768.
// In other words, produces an integer-resolution sine wave with amplitude
// 32768±32767 (instead of 0±1) and period 65536 (instead of 2p).

//uint32_t sin16(uint32_t x);

////////////////////////////////////////////////////////////////////////////////
// cos16()
// Produces low resolution cosine values as quickly as possible. Properties are
// identical to those of sin16.

//uint32_t cos16(uint32_t x);

////////////////////////////////////////////////////////////////////////////////
// tan16()
// Produces low resolution tangent values as quickly as possible. Takes input of
// uint32_t x and returns uint32_t 799*tangent(2px/65536)+32768. In other words,
// produces an integer-resolution tangent plot centered around 32768 and 799x
// normal scale (because 1x scale would be useless at integer resolution) and
// period 65536 (instead of 2p).

//uint32_t tan16(uint32_t x);
