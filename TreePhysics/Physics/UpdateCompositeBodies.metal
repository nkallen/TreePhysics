#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
using namespace metal;

inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody,
                              const CompositeBodyStruct childCompositeBodies[5])
{
    CompositeBodyStruct compositeBody;

    compositeBody.mass = rigidBody.mass;
    compositeBody.force = rigidBody.force;
    compositeBody.torque = rigidBody.torque;
    compositeBody.centerOfMass = rigidBody.mass * rigidBody.centerOfMass;
    compositeBody.position = rigidBody.position;

    for (size_t i = 0; i < rigidBody.childCount; i++) {
        CompositeBodyStruct childCompositeBody = childCompositeBodies[i];

        compositeBody.mass += childCompositeBody.mass;
        compositeBody.force += childCompositeBody.force;
        compositeBody.torque += cross(childCompositeBody.position - rigidBody.position, childCompositeBody.force) + childCompositeBody.torque;
        compositeBody.centerOfMass += childCompositeBody.mass * childCompositeBody.centerOfMass;
    }
    compositeBody.centerOfMass /= compositeBody.mass;

    compositeBody.inertiaTensor = rigidBody.inertiaTensor - rigidBody.mass * sqr(crossMatrix(rigidBody.centerOfMass - compositeBody.centerOfMass));

    for (ushort i = 0; i < rigidBody.childCount; i++) {
        CompositeBodyStruct childCompositeBody = childCompositeBodies[i];

        compositeBody.inertiaTensor += childCompositeBody.inertiaTensor - childCompositeBody.mass * sqr(crossMatrix(childCompositeBody.centerOfMass - compositeBody.centerOfMass));
    }

    
    return compositeBody;
}

// Just a duplicate for one child; eventually we'll unroll all of these.
inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody,
                              const CompositeBodyStruct childCompositeBody)
{
    CompositeBodyStruct compositeBody;
    
    compositeBody.mass = rigidBody.mass;
    compositeBody.force = rigidBody.force;
    compositeBody.torque = rigidBody.torque;
    compositeBody.centerOfMass = rigidBody.mass * rigidBody.centerOfMass;
    compositeBody.position = rigidBody.position;
    
    compositeBody.mass += childCompositeBody.mass;
    compositeBody.force += childCompositeBody.force;
    compositeBody.torque += cross(childCompositeBody.position - rigidBody.position, childCompositeBody.force) + childCompositeBody.torque;
    compositeBody.centerOfMass += childCompositeBody.mass * childCompositeBody.centerOfMass;
    compositeBody.centerOfMass /= compositeBody.mass;

    compositeBody.inertiaTensor = rigidBody.inertiaTensor - rigidBody.mass * sqr(crossMatrix(rigidBody.centerOfMass - compositeBody.centerOfMass));

    compositeBody.inertiaTensor += childCompositeBody.inertiaTensor - childCompositeBody.mass * sqr(crossMatrix(childCompositeBody.centerOfMass - compositeBody.centerOfMass));

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

inline uint
rigidBody_childCompositeBodies(
                               const RigidBodyStruct rigidBody,
                               device CompositeBodyStruct * compositeBodies,
                               int minIdInThreadGroup,
                               CompositeBodyStruct childCompositeBodies[5])
{
    uint missing = 0;
    for (ushort i = 0; i < rigidBody.childCount; i++) {
        int childId = rigidBody.childIds[i];
        if (childId >= minIdInThreadGroup) {
            missing++;
        } else {
            childCompositeBodies[i] = compositeBodies[childId];
        }
    }
    return missing;
}

inline void
rigidBody_childCompositeBodies(
                               const RigidBodyStruct rigidBody,
                               device CompositeBodyStruct * compositeBodies,
                               ushort minIdInThreadGroup,
                               threadgroup CompositeBodyStruct * compositeBodiesCache,
                               CompositeBodyStruct childCompositeBodies[5])
{
    for (ushort i = 0; i < rigidBody.childCount; i++) {
        int childId = rigidBody.childIds[i];
        if (childId >= minIdInThreadGroup) {
            ushort lid = childId - minIdInThreadGroup;
            childCompositeBodies[i] = compositeBodiesCache[lid];
        } else {
            childCompositeBodies[i] = compositeBodies[childId];
        }
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
                      constant uint * gridOrigin [[ buffer(BufferIndexGridOrigin) ]],
                      uint gid [[ thread_position_in_grid ]])
{
    int id = *gridOrigin + gid;
    RigidBodyStruct rigidBody = rigidBodies[id];
    CompositeBodyStruct compositeBody = rigidBody_updateCompositeBody(rigidBody, rigidBodies, compositeBodies);
    compositeBodies[id] = compositeBody;

    rigidBody_climb(rigidBody, compositeBody, rigidBodies, compositeBodies);
}
