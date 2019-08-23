#include "Half.h"

// MARK: Half conversions

static float inline F16ToF32(const __fp16 *address) {
    return *address;
}

float float32_from_float16(uint16_t i) {
    return F16ToF32((__fp16 *)&i);
}

static inline void F32ToF16(float F32, __fp16 *F16Ptr) {
    *F16Ptr = F32;
}

uint16_t float16_from_float32(float f) {
    uint16_t f16;
    F32ToF16(f, (__fp16 *)&f16);
    return f16;
}
