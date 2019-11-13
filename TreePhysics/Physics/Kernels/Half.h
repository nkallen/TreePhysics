#ifndef Math_h
#define Math_h

#include <stdio.h>

// Given a uint16_t encoded as a 16-bit float, returns a 32-bit float
float float32_from_float16(uint16_t i);

// Given a 32-bit float, returns a uint16_t encoded as a 16-bit float
uint16_t float16_from_float32(float f);

typedef uint16_t half;

// NOTE: the alignment is important for interop between metal and swift; metal half3 has alignment 8, but by a default struct like this would have alignment 2.
typedef struct {
    half x; half y; half z; half w;
} __attribute__((aligned(8))) vector_half3;

typedef struct {
    half x; half y; half z;
} packed_half3;

typedef struct {
    float x; float y; float z;
} packed_float3;

typedef struct {
    half x; half y; half z; half w;
} __attribute__((aligned(8))) vector_half4;

typedef struct { vector_half3 columns[3]; } matrix_half3x3;

typedef matrix_half3x3 half3x3;
typedef vector_half4 simd_quath;

#endif /* Math_h */
