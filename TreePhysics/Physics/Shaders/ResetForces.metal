#include <metal_stdlib>
#import "ShaderTypes.h"

using namespace metal;

kernel void
resetForces(
                      device RigidBodyStruct * rigidBodies [[ buffer(BufferIndexRigidBodies) ]],
                      uint gid [[ thread_position_in_grid ]])
{
    RigidBodyStruct rigidBody = rigidBodies[gid];

    if (rigidBody.parentId != -1) {
        rigidBody.force = half3(0);
        rigidBody.torque = half3(0);
    }
    rigidBodies[gid] = rigidBody;
}
