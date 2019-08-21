#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
#import "Print.metal"
using namespace metal;

constant int rangeCount [[ function_constant(FunctionConstantIndexRangeCount) ]];

inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody,
                              const CompositeBodyStruct childCompositeBodies[3],
                              thread Debug & debug)
{
    float mass = rigidBody.mass;
    half3 force = rigidBody.force;
    half3 torque = rigidBody.torque;
    float3 centerOfMass = rigidBody.mass * (float3)rigidBody.centerOfMass;
    half3 position = rigidBody.position;

    // debug << "update composite body N child\n";
    // debug << "centerOfMass=" << centerOfMass << "\n";
    // debug << "mass=" << mass << "\n";

    for (ushort i = 0; i < rigidBody.childCount; i++) {
        CompositeBodyStruct childCompositeBody = childCompositeBodies[i];

        mass += childCompositeBody.mass;
        force += childCompositeBody.force;
        torque += cross(childCompositeBody.position - position, childCompositeBody.force) + childCompositeBody.torque;
        centerOfMass += childCompositeBody.mass * (float3)childCompositeBody.centerOfMass;
        // debug << "centerOfMass=" << centerOfMass << "\n";
        // debug << "mass=" << mass << "\n";
    }
    centerOfMass /= mass;
    // debug << "centerOfMass=" << centerOfMass << "\n";

    float3x3 inertiaTensor = rigidBody.inertiaTensor - rigidBody.mass * sqr(crossMatrix((float3)rigidBody.centerOfMass - centerOfMass));

    for (ushort i = 0; i < rigidBody.childCount; i++) {
        CompositeBodyStruct childCompositeBody = childCompositeBodies[i];

        inertiaTensor += childCompositeBody.inertiaTensor - childCompositeBody.mass * sqr(crossMatrix((float3)childCompositeBody.centerOfMass - centerOfMass));
    }

    CompositeBodyStruct compositeBody = {
        .mass = mass,
        .force = force,
        .torque = torque,
        .centerOfMass = half3(centerOfMass),
        .position = position,
        .inertiaTensor = inertiaTensor
    };

    return compositeBody;
}

inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody,
                              const CompositeBodyStruct childCompositeBody,
                              thread Debug & debug)
{
    float mass = rigidBody.mass;
    half3 force = rigidBody.force;
    half3 torque = rigidBody.torque;
    float3 centerOfMass = mass * (float3)rigidBody.centerOfMass;
    half3 position = rigidBody.position;

    // debug << "update composite body 1 child\n";
    // debug << "rigidBody.centerOfMass=" << rigidBody.centerOfMass << "\n";
    // debug << "centerOfMass=" << centerOfMass << "\n";
    // debug << "mass=" << mass << "\n";

    mass += childCompositeBody.mass;
    // debug << "mass=" << mass << "\n";
    force += childCompositeBody.force;
    torque += cross(childCompositeBody.position - position, childCompositeBody.force) + childCompositeBody.torque;
    centerOfMass += childCompositeBody.mass * (float3)childCompositeBody.centerOfMass;
    // debug << "centerOfMass=" << centerOfMass << "\n";
    centerOfMass /= mass;
    // debug << "centerOfMass=" << centerOfMass << "\n";

    float3x3 inertiaTensor = rigidBody.inertiaTensor - rigidBody.mass * sqr(crossMatrix((float3)rigidBody.centerOfMass - centerOfMass));

    inertiaTensor += childCompositeBody.inertiaTensor - childCompositeBody.mass * sqr(crossMatrix((float3)childCompositeBody.centerOfMass - centerOfMass));

    CompositeBodyStruct compositeBody = {
        .mass = mass,
        .force = force,
        .torque = torque,
        .centerOfMass = (half3)centerOfMass,
        .position = position,
        .inertiaTensor = inertiaTensor
    };

    return compositeBody;
}

inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody,
                              thread Debug & debug)
{
    // debug << "update composite body 0 child\n";
    // debug << "centerOfMass=" << rigidBody.centerOfMass << "\n";
    // debug << "mass=" << rigidBody.mass << "\n";

    CompositeBodyStruct compositeBody = {
        .mass = rigidBody.mass,
        .force = rigidBody.force,
        .torque = rigidBody.torque,
        .centerOfMass = rigidBody.centerOfMass,
        .position = rigidBody.position,
        .inertiaTensor = rigidBody.inertiaTensor
    };

    return compositeBody;
}

inline void
rigidBody_childCompositeBodies(
                               const RigidBodyStruct rigidBody,
                               device CompositeBodyStruct * compositeBodies,
                               CompositeBodyStruct childCompositeBodies[5],
                               thread Debug & debug)
{
    for (int i = 0; i < rigidBody.childCount; i++) {
        int childId = rigidBody.childIds[i];
        // debug << "childId: " << childId << "\n";
        childCompositeBodies[i] = compositeBodies[childId];
        // debug << "child.centerOfMAss: " << childCompositeBodies[i].centerOfMass << "\n";
    }
}

inline void
rigidBody_climb(const RigidBodyStruct rigidBody,
                const CompositeBodyStruct compositeBody,
                device RigidBodyStruct * rigidBodies,
                device CompositeBodyStruct * compositeBodies,
                thread Debug & debug)
{
    CompositeBodyStruct currentCompositeBody = compositeBody;

    for (ushort i = 0; i < rigidBody.climberCount; i++) {
        int id = rigidBody.climberOffset + i;
        // debug << "climbing up: " << id << "\n";

        RigidBodyStruct currentRigidBody = rigidBodies[id];
        currentCompositeBody = rigidBody_updateCompositeBody(currentRigidBody, currentCompositeBody, debug);
        compositeBodies[id] = currentCompositeBody;
    }
}

inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody,
                              device RigidBodyStruct * rigidBodies,
                              device CompositeBodyStruct * compositeBodies,
                              thread Debug & debug)
{
    if (rigidBody.childCount == 0) {
        // debug << "no children: " << "\n";
        return rigidBody_updateCompositeBody(rigidBody, debug);
    } else {
        // debug << "child count: " << rigidBody.childCount << "\n";
        CompositeBodyStruct childCompositeBodies[3];
        rigidBody_childCompositeBodies(rigidBody, compositeBodies, childCompositeBodies, debug);
        return rigidBody_updateCompositeBody(rigidBody, childCompositeBodies, debug);
    }
}

kernel void
updateCompositeBodies(
                      device RigidBodyStruct * rigidBodies [[ buffer(BufferIndexRigidBodies) ]],
                      device CompositeBodyStruct * compositeBodies [[ buffer(BufferIndexCompositeBodies) ]],
                      constant int2 * ranges [[ buffer(BufferIndexRanges) ]],
                      uint gid [[ thread_position_in_grid ]],
                      device char * buf [[ buffer(BufferIndexDebugString) ]])
{
    Debug debug = Debug(buf + gid*8192, 8192);

    for (ushort i = 0; i < rangeCount; i++) {
        // debug << "i: " << i << "\n";
        int2 range = ranges[i];
        int lowerBound = range.x;
        int upperBound = range.y;
        // debug << "range: (" << lowerBound << ", " << upperBound << ")\n";
        if ((int)gid < upperBound - lowerBound) {
            int id = lowerBound + gid;

//            debug << "id: " << id << "\n";

            RigidBodyStruct rigidBody = rigidBodies[id];
            CompositeBodyStruct compositeBody = rigidBody_updateCompositeBody(rigidBody, rigidBodies, compositeBodies, debug);
            compositeBodies[id] = compositeBody;

            rigidBody_climb(rigidBody, compositeBody, rigidBodies, compositeBodies, debug);
        }
        threadgroup_barrier(mem_flags::mem_device);
    }
}
