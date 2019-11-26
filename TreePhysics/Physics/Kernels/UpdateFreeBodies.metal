#include <metal_stdlib>
using namespace metal;

kernel void
updateFreeBodies(
           device uint           *free_index,
           device half           *free_mass,
           device packed_half3   *free_force,
           device packed_half3   *free_torque,
           device InertiaTensor  *free_inertiaTensor,
           uint gid [[ thread_position_in_grid ]])
{
    const uint id = args.freeIndex[gid];

}
