#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
#import "Print.metal"
using namespace metal;

constant int rangeCount [[ function_constant(FunctionConstantIndexRangeCount) ]];

inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody,
                              const CompositeBodyStruct childCompositeBodies[3], thread Debug & debug)
{
    half mass = half(rigidBody.mass);
    half3 force = half3(rigidBody.force);
    half3 torque = half3(rigidBody.torque);
    half3 centerOfMass = rigidBody.mass * rigidBody.centerOfMass;
    half3 position = half3(rigidBody.position);

    debug << "mass: " << mass << "\n";
    
    for (ushort i = 0; i < rigidBody.childCount; i++) {
        CompositeBodyStruct childCompositeBody = childCompositeBodies[i];
        
        mass += half(childCompositeBody.mass);
        force += half3(childCompositeBody.force);
        torque += cross(half3(childCompositeBody.position) - position, half3(childCompositeBody.force)) + half3(childCompositeBody.torque);
        centerOfMass += childCompositeBody.mass * childCompositeBody.centerOfMass;
    }
    centerOfMass /= mass;

    debug << "mass: " << mass << "\n";

    half3x3 inertiaTensor = rigidBody.inertiaTensor - rigidBody.mass * sqr(crossMatrix(rigidBody.centerOfMass - centerOfMass));
    
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
    half mass = rigidBody.mass;
    half3 force = rigidBody.force;
    half3 torque = rigidBody.torque;
    half3 centerOfMass = mass * rigidBody.centerOfMass;
    half3 position = rigidBody.position;
    
    mass += childCompositeBody.mass;
    force += childCompositeBody.force;
    torque += cross(childCompositeBody.position - position, childCompositeBody.force) + childCompositeBody.torque;
    centerOfMass += childCompositeBody.mass * childCompositeBody.centerOfMass;
    centerOfMass /= mass;
    
    half3x3 inertiaTensor = rigidBody.inertiaTensor - rigidBody.mass * sqr(crossMatrix(rigidBody.centerOfMass - centerOfMass));
    
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

inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody)
{
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
                               CompositeBodyStruct childCompositeBodies[5], thread Debug & debug)
{
    for (int i = 0; i < rigidBody.childCount; i++) {
        int childId = rigidBody.childIds[i];
        debug << "loading child id: " << childId << "\n";
        childCompositeBodies[i] = compositeBodies[childId];
    }
}

inline void
rigidBody_climb(const RigidBodyStruct rigidBody,
                const CompositeBodyStruct compositeBody,
                device RigidBodyStruct * rigidBodies,
                device CompositeBodyStruct * compositeBodies, thread Debug & debug)
{
    CompositeBodyStruct currentCompositeBody = compositeBody;

    debug << "in climb -- " << "\n";

    for (ushort i = 0; i < rigidBody.climberCount; i++) {
        debug << "climbing iteration: " << i << " and id: " << rigidBody.climberOffset + i << "\n";
        RigidBodyStruct currentRigidBody = rigidBodies[rigidBody.climberOffset + i];
        currentCompositeBody = rigidBody_updateCompositeBody(currentRigidBody, currentCompositeBody);
        debug << "set mass to: " << currentCompositeBody.mass << "\n";
        compositeBodies[rigidBody.climberOffset + i] = currentCompositeBody;
    }
}

inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody,
                              device RigidBodyStruct * rigidBodies,
                              device CompositeBodyStruct * compositeBodies, thread Debug & debug)
{
    debug << "childcount = " << rigidBody.childCount << "\n";
    if (rigidBody.childCount == 0) {
        return rigidBody_updateCompositeBody(rigidBody);
    } else {
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
                      device char *buf [[ buffer(BufferIndexDebugString) ]])
{
    Debug debug = Debug(buf + (gid * 1024), 1024);
    debug << "gid: " << gid << "\n";

    for (ushort i = 0; i < rangeCount; i++) {
        int2 range = ranges[i];
        debug << "range " << i << " " << range << "\n";
        int lowerBound = range.x;
        int upperBound = range.y;
        if ((int)gid < upperBound - lowerBound) {
            int id = lowerBound + gid;
            debug << "id: " << id << "\n";

            RigidBodyStruct rigidBody = rigidBodies[id];
            CompositeBodyStruct compositeBody = rigidBody_updateCompositeBody(rigidBody, rigidBodies, compositeBodies, debug);
            compositeBodies[id] = compositeBody;
            
            rigidBody_climb(rigidBody, compositeBody, rigidBodies, compositeBodies, debug);
        }
        debug << "---------------------\n";
        threadgroup_barrier(mem_flags::mem_device);
    }
}
