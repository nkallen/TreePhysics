#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
#import "Debug.metal"
using namespace metal;

constant int rangeCount [[ function_constant(FunctionConstantIndexRangeCount) ]];

inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody,
                              const CompositeBodyStruct childCompositeBodies[3])
{
    float mass = rigidBody.mass;
    float3 force = rigidBody.force;
    float3 torque = rigidBody.torque;
    float3 centerOfMass = rigidBody.mass * (float3)rigidBody.centerOfMass;
    float3 pivot = rigidBody.pivot;

    for (ushort i = 0; i < rigidBody.childCount; i++) {
        CompositeBodyStruct childCompositeBody = childCompositeBodies[i];

        mass += childCompositeBody.mass;
        force += childCompositeBody.force;
        torque += cross(childCompositeBody.pivot - pivot, childCompositeBody.force) + childCompositeBody.torque;
        centerOfMass += childCompositeBody.mass * (float3)childCompositeBody.centerOfMass;
    }
    centerOfMass /= mass;

    float3x3 inertiaTensor = rigidBody.inertiaTensor - rigidBody.mass * sqr(crossMatrix((float3)rigidBody.centerOfMass - centerOfMass));

    for (ushort i = 0; i < rigidBody.childCount; i++) {
        CompositeBodyStruct childCompositeBody = childCompositeBodies[i];

        inertiaTensor += childCompositeBody.inertiaTensor - childCompositeBody.mass * sqr(crossMatrix((float3)childCompositeBody.centerOfMass - centerOfMass));
    }

    CompositeBodyStruct compositeBody = {
        .mass = mass,
        .force = force,
        .torque = torque,
        .centerOfMass = float3(centerOfMass),
        .pivot = pivot,
        .inertiaTensor = inertiaTensor
    };

    return compositeBody;
}

inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody,
                              const CompositeBodyStruct childCompositeBody)
{
    float mass = rigidBody.mass;
    float3 force = rigidBody.force;
    float3 torque = rigidBody.torque;
    float3 centerOfMass = mass * (float3)rigidBody.centerOfMass;
    float3 pivot = rigidBody.pivot;

    mass += childCompositeBody.mass;
    force += childCompositeBody.force;
    torque += cross(childCompositeBody.pivot - pivot, childCompositeBody.force) + childCompositeBody.torque;
    centerOfMass += childCompositeBody.mass * (float3)childCompositeBody.centerOfMass;
    centerOfMass /= mass;

    float3x3 inertiaTensor = rigidBody.inertiaTensor - rigidBody.mass * sqr(crossMatrix((float3)rigidBody.centerOfMass - centerOfMass));

    inertiaTensor += childCompositeBody.inertiaTensor - childCompositeBody.mass * sqr(crossMatrix((float3)childCompositeBody.centerOfMass - centerOfMass));

    CompositeBodyStruct compositeBody = {
        .mass = mass,
        .force = force,
        .torque = torque,
        .centerOfMass = (float3)centerOfMass,
        .pivot = pivot,
        .inertiaTensor = inertiaTensor
    };

    return compositeBody;
}

inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody)
{
    CompositeBodyStruct compositeBody = {
        .mass = rigidBody.mass,
        .force = rigidBody.force,
        .torque = rigidBody.torque,
        .centerOfMass = rigidBody.centerOfMass,
        .pivot = rigidBody.pivot,
        .inertiaTensor = rigidBody.inertiaTensor
    };

    return compositeBody;
}

inline void
rigidBody_childCompositeBodies(
                               const RigidBodyStruct rigidBody,
                               device CompositeBodyStruct * compositeBodies,
                               CompositeBodyStruct childCompositeBodies[5])
{
    for (int i = 0; i < rigidBody.childCount; i++) {
        int childId = rigidBody.childIds[i];
        childCompositeBodies[i] = compositeBodies[childId];
    }
}

inline void
rigidBody_climb(const RigidBodyStruct rigidBody,
                const CompositeBodyStruct compositeBody,
                device RigidBodyStruct * rigidBodies,
                device CompositeBodyStruct * compositeBodies)
{
    CompositeBodyStruct currentCompositeBody = compositeBody;

    for (ushort i = 0; i < rigidBody.climberCount; i++) {
        int id = rigidBody.climberOffset + i;

        RigidBodyStruct currentRigidBody = rigidBodies[id];
        currentCompositeBody = rigidBody_updateCompositeBody(currentRigidBody, currentCompositeBody);
        compositeBodies[id] = currentCompositeBody;
    }
}

inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody,
                              device RigidBodyStruct * rigidBodies,
                              device CompositeBodyStruct * compositeBodies)
{
    if (rigidBody.childCount == 0) {
        return rigidBody_updateCompositeBody(rigidBody);
    } else {
        CompositeBodyStruct childCompositeBodies[3];
        rigidBody_childCompositeBodies(rigidBody, compositeBodies, childCompositeBodies);
        return rigidBody_updateCompositeBody(rigidBody, childCompositeBodies);
    }
}

kernel void
updateCompositeBodies(
                      device RigidBodyStruct * rigidBodies [[ buffer(BufferIndexRigidBodies) ]],
                      device CompositeBodyStruct * compositeBodies [[ buffer(BufferIndexCompositeBodies) ]],
                      constant int2 * ranges [[ buffer(BufferIndexRanges) ]],
                      uint gid [[ thread_position_in_grid ]])
{
    for (ushort i = 0; i < rangeCount; i++) {
        int2 range = ranges[i];
        int lowerBound = range.x;
        int upperBound = range.y;
        if ((int)gid < upperBound - lowerBound) {
            int id = lowerBound + gid;

            RigidBodyStruct rigidBody = rigidBodies[id];
            CompositeBodyStruct compositeBody = rigidBody_updateCompositeBody(rigidBody, rigidBodies, compositeBodies);
            compositeBodies[id] = compositeBody;

            rigidBody_climb(rigidBody, compositeBody, rigidBodies, compositeBodies);
        }
        threadgroup_barrier(mem_flags::mem_device);
    }
}
