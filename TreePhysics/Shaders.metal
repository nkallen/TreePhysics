#include <metal_stdlib>
using namespace metal;

kernel void diagonalize(
                        constant float3x3 & matrix [[ buffer(0) ]],
                        device float3 *out [[ buffer(1) ]],
                        uint2 gid [[ thread_position_in_grid ]])
{
    out[gid.y] = float3(0.1, 0.2, 0.3);
}
