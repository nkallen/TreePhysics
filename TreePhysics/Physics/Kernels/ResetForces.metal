#include <metal_stdlib>
#import "ShaderTypes.h"

using namespace metal;

kernel void
resetForces(
            device packed_half3   *force,
            device packed_half3   *torque,
            uint gid [[ thread_position_in_grid ]])
{
    force[gid] = packed_half3(0);
    torque[gid] = packed_half3(0);
}
