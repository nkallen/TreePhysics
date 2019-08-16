#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
using namespace metal;

constant int rangeCount [[ function_constant(FunctionConstantIndexRangeCount) ]];

inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody,
                              const CompositeBodyStruct childCompositeBodies[5])
{
    float mass = rigidBody.mass;
    float3 force = rigidBody.force;
    float3 torque = rigidBody.torque;
    float3 centerOfMass = rigidBody.mass * rigidBody.centerOfMass;
    float3 position = rigidBody.position;

    for (ushort i = 0; i < rigidBody.childCount; i++) {
        CompositeBodyStruct childCompositeBody = childCompositeBodies[i];

        mass += childCompositeBody.mass;
        force += childCompositeBody.force;
        torque += cross(childCompositeBody.position - rigidBody.position, childCompositeBody.force) + childCompositeBody.torque;
        centerOfMass += childCompositeBody.mass * childCompositeBody.centerOfMass;
    }
    centerOfMass /= mass;

    float3x3 inertiaTensor = rigidBody.inertiaTensor - rigidBody.mass * sqr(crossMatrix(rigidBody.centerOfMass - centerOfMass));

    for (ushort i = 0; i < rigidBody.childCount; i++) {
        CompositeBodyStruct childCompositeBody = childCompositeBodies[i];

        inertiaTensor += childCompositeBody.inertiaTensor - childCompositeBody.mass * sqr(crossMatrix(childCompositeBody.centerOfMass - centerOfMass));
    }

    CompositeBodyStruct compositeBody = {
        .mass = mass,
        .force = force,
        .torque = torque,
        .centerOfMass = centerOfMass,
        .position = position,
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
    float3 centerOfMass = rigidBody.mass * rigidBody.centerOfMass;
    float3 position = rigidBody.position;

    mass += childCompositeBody.mass;
    force += childCompositeBody.force;
    torque += cross(childCompositeBody.position - rigidBody.position, childCompositeBody.force) + childCompositeBody.torque;
    centerOfMass += childCompositeBody.mass * childCompositeBody.centerOfMass;
    centerOfMass /= mass;

    float3x3 inertiaTensor = rigidBody.inertiaTensor - rigidBody.mass * sqr(crossMatrix(rigidBody.centerOfMass - centerOfMass));

    inertiaTensor += childCompositeBody.inertiaTensor - childCompositeBody.mass * sqr(crossMatrix(childCompositeBody.centerOfMass - centerOfMass));

    CompositeBodyStruct compositeBody = {
        .mass = mass,
        .force = force,
        .torque = torque,
        .centerOfMass = centerOfMass,
        .position = position,
        .inertiaTensor = inertiaTensor
    };

    return compositeBody;
}

inline void
rigidBody_childRigidBodies(
                           const RigidBodyStruct rigidBody,
                           device RigidBodyStruct * rigidBodies,
                           RigidBodyStruct childRigidBodies[5])
{
    for (ushort i = 0; i < rigidBody.childCount; i++) {
        int childId = rigidBody.childIds[i];
        childRigidBodies[i] = childRigidBodies[childId];
    }
}


inline void
rigidBody_childCompositeBodies(
                               const RigidBodyStruct rigidBody,
                               device CompositeBodyStruct * compositeBodies,
                               CompositeBodyStruct childCompositeBodies[5])
{
    for (int i = 0; i < 5; i++) {
        int childId = rigidBody.childIds[i];
        childCompositeBodies[i] = compositeBodies[childId];
    }
}

inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody,
                              device RigidBodyStruct * rigidBodies,
                              device CompositeBodyStruct * compositeBodies)
{
    CompositeBodyStruct childCompositeBodies[5];
    rigidBody_childCompositeBodies(rigidBody, compositeBodies, childCompositeBodies);
    return rigidBody_updateCompositeBody(rigidBody, childCompositeBodies);
}

void
rigidBody_climb(const RigidBodyStruct rigidBody,
                const CompositeBodyStruct compositeBody,
                device RigidBodyStruct * rigidBodies,
                device CompositeBodyStruct * compositeBodies)
{
    RigidBodyStruct currentRigidBody = rigidBody;
    CompositeBodyStruct currentCompositeBody = compositeBody;

    while (currentRigidBody.parentId != -1) {
        int parentId = currentRigidBody.parentId;
        currentRigidBody = rigidBodies[parentId];
        if (currentRigidBody.childCount == 1) {
            currentCompositeBody = rigidBody_updateCompositeBody(currentRigidBody, currentCompositeBody);
            compositeBodies[parentId] = currentCompositeBody;
        } else {
            return;
        }
    }
}

kernel void
updateCompositeBodies(
                      device RigidBodyStruct * rigidBodies [[ buffer(BufferIndexRigidBodies) ]],
                      device CompositeBodyStruct * compositeBodies [[ buffer(BufferIndexCompositeBodies) ]],
                      constant int2 * ranges [[ buffer(BufferIndexRanges) ]],
                      uint gid [[ thread_position_in_grid ]])
{
    for (int i = 0; i < rangeCount; i++) {
        int2 range = ranges[i];
        int lowerBound = range.x;
        int upperBound = range.y;
        if ((int)gid >= 0 && (int)gid < upperBound - lowerBound) {
            int id = lowerBound + gid;

            RigidBodyStruct rigidBody = rigidBodies[id];
            CompositeBodyStruct compositeBody = rigidBody_updateCompositeBody(rigidBody, rigidBodies, compositeBodies);
            compositeBodies[id] = compositeBody;

            rigidBody_climb(rigidBody, compositeBody, rigidBodies, compositeBodies);
        }
        threadgroup_barrier(mem_flags::mem_device);
    }
}
