#include <metal_stdlib>
#include "ShaderTypes.h"
#include "Math.metal"

using namespace metal;

struct arguments {
    command_buffer        cmd_buffer;
    device uint           *freeBodyIndex;
    device uint           &freeBodyCount;
    device half           *mass;
    device packed_half3   *force;
    device packed_half3   *torque;
    device InertiaTensor  *freeInertiaTensor;

    device packed_float3  *localInertiaTensor;
    device packed_float3   *centerOfMass;
    device quatf          *orientation;
    device packed_half3   *angularMomentum;
    device packed_half3   *velocity;
    device packed_half3   *acceleration;
    device packed_half3   *angularVelocity;
    device InertiaTensor   *inertiaTensor;
};

kernel void
encodeUpdateFreeBodies(
                       device arguments      &args,
                       device const float    &time,
                       uint gid [[ thread_position_in_grid ]])
{
    compute_command cmd(args.cmd_buffer, gid);
    cmd.concurrent_dispatch_threads(uint3(args.freeBodyCount, 1, 1), uint3(1024, 1, 1));
}

kernel void
updateFreeBodies(
                 device arguments      &args,
                 device const float    &time,
                 uint gid [[ thread_position_in_grid ]])
{
    const uint id = args.freeBodyIndex[gid];

    float3 acceleration = (float3)args.force[id] / (float)args.mass[id];

    float3 angularMomentum = (float3)args.angularMomentum[id] + time * (float3)args.torque[id];

    float3 velocity = (float3)args.velocity[id];
    velocity += time * acceleration;
    float3 angularVelocity = inverse(float3x3_from_inertiaTensor(args.inertiaTensor[id])) * angularMomentum;

    float3 centerOfMass = (float3)args.centerOfMass[id] + time * velocity;
    quatf angularVelocityQuat = quaternion(angularVelocity, 0);
    quatf orientation = (quatf)args.orientation[id];
    orientation += time/2.0 * quat_multiply(angularVelocityQuat, orientation);
    orientation = normalize(orientation);

    float3 localInertiaTensor = args.localInertiaTensor[id];
    float3x3 inertiaTensor = quat_rotateTensor(orientation, localInertiaTensor);

    args.velocity[id]        = (packed_half3)velocity;
    args.acceleration[id]    = (packed_half3)acceleration;
    args.angularMomentum[id] = (packed_half3)angularMomentum;
    args.angularVelocity[id] = (packed_half3)angularVelocity;
    args.centerOfMass[id]    = (packed_float3)centerOfMass;
    args.orientation[id]     = (quatf)orientation;
    args.inertiaTensor[id]   = inertiaTensor_from_float3x3(inertiaTensor);
}
