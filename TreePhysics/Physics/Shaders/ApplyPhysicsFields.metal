#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"

using namespace metal;

constant half a = 0.05;
constant half b = 0.01;
constant half c = 0.1;

kernel void
applyPhysicsFields(
            constant PhysicsFieldStruct * physicsField [[ buffer(BufferIndexPhysicsField) ]],
            device RigidBodyStruct * rigidBodies [[ buffer(BufferIndexRigidBodies) ]],
            uint gid [[ thread_position_in_grid ]])
{
    RigidBodyStruct rigidBody = rigidBodies[gid];

    half3 delta = physicsField->position - rigidBody.position;
    half distance = length(delta);
    half3 direction = normalize(delta);
    // NOTE: this is a bell-shaped curve, so objects very far and very near are weakly affected
    half3 force = direction * a * pow(M_E_H, -sqr(distance - b)/(2*c));

    rigidBody.force = force;

    rigidBodies[gid] = rigidBody;
}
