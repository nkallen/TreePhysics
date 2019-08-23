#include <metal_stdlib>
#import "ShaderTypes.h"

using namespace metal;

//constant bool med_quality_defined = is_function_constant_defined(med_quality);

kernel void
resetForces(
            device RigidBodyStruct * rigidBodies [[ buffer(BufferIndexRigidBodies) ]],
            uint gid [[ thread_position_in_grid ]])
{
    RigidBodyStruct rigidBody = rigidBodies[gid];

    rigidBody.force = float3(0);
    rigidBody.torque = float3(0);
    rigidBodies[gid] = rigidBody;
}
