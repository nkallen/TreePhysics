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

    device float3         *localInertiaTensor;
    device packed_half3   *centerOfMass;
    device quath          *orientation;
    device packed_half3   *angularMomentum;
    device packed_half3   *velocity;
    device packed_half3   *acceleration;
    device packed_half3   *angularVelocity;
    device InertiaTensor   *inertiaTensor;
};

kernel void
encodeUpdateFreeBodies(
                       device arguments      &args,
//                       device const float    &time,
                       uint gid [[ thread_position_in_grid ]])
{
    compute_command cmd(args.cmd_buffer, gid);
    cmd.concurrent_dispatch_threads(uint3(args.freeBodyCount, 1, 1), uint3(1024, 1, 1));
}

kernel void
updateFreeBodies(
                 device arguments      &args,
//                 device const float    &time,
                 uint gid [[ thread_position_in_grid ]])
{
    float time = 1.0 / 60;

    const uint id = args.freeBodyIndex[gid];

    float3 acceleration = (float3)args.force[id] / args.mass[id];

    float3 angularMomentum = (float3)args.angularMomentum[id] + time * (float3)args.torque[id];

    float3 velocity = (float3)args.velocity[id] + time * acceleration;
    float3 angularVelocity = float3x3_from_inertiaTensor(args.freeInertiaTensor[id]) * angularMomentum;

    float3 centerOfMass = (float3)args.centerOfMass[id] + time * velocity;
    quatf angularVelocityQuat = quaternion(angularVelocity, 0);
    quatf orientation = (quatf)args.orientation[id];
    orientation = orientation + time/2 * angularVelocityQuat * orientation;
    orientation = normalize(orientation);

    float3 localInertiaTensor = args.localInertiaTensor[id];

    float3x3 inertiaTensor = float3x3_from_quat(orientation) * float3x3(localInertiaTensor.x, 0, 0, 0, localInertiaTensor.y, 0, 0, 0, localInertiaTensor.z) * transpose(float3x3_from_quat(orientation));

    args.velocity[id]        = (packed_half3)velocity;
    args.acceleration[id]    = (packed_half3)acceleration;
    args.angularMomentum[id] = (packed_half3)angularMomentum;
    args.angularVelocity[id] = (packed_half3)angularVelocity;
    args.centerOfMass[id]    = (packed_half3)centerOfMass;
    args.orientation[id]     = (quath)orientation;
    args.inertiaTensor[id]   = inertiaTensor_from_float3x3(inertiaTensor);
}
