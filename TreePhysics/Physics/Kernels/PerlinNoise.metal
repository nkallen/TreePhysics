#include <metal_stdlib>
#include "PerlinNoise.h"

using namespace metal;

inline float random(float2 st) {
    return fract(sin(dot(st,
                         float2(12.9898,78.233))) *
                 43758.5453123);
}

// Based on Morgan McGuire @morgan3d
// https://www.shadertoy.com/view/4dS3Wd
inline float noise(const float2 st) {
    const float2 i = floor(st);
    const float2 f = fract(st);

    // Four corners in 2D of a tile
    const float a = random(i);
    const float b = random(i + float2(1.0, 0.0));
    const float c = random(i + float2(0.0, 1.0));
    const float d = random(i + float2(1.0, 1.0));

    float2 u = f * f;
    u *= (3.0 - 2.0 * f);

    return mix(a, b, u.x) +
    (c - a) * u.y * (1.0 - u.x) +
    (d - b) * u.x * u.y;
}

inline PerlinNoise::PerlinNoise(float frequency, int octaves, float persistence, float lacunarity, int seed) {
    this->frequency = frequency;
    this->octaves = octaves;
    this->persistence = persistence;
    this->lacunarity = lacunarity;
}

inline float PerlinNoise::value(float2 st) {
    float value = 0.0;

    for (int i = 0; i < octaves; i++) {
        float amplitude = 1;

        value += amplitude * noise(frequency*st+seed);
        st *= lacunarity;
        amplitude *= persistence;
    }
    return value;
}
