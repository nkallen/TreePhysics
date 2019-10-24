#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"

using namespace metal;

constant float a = 0.05;
constant float b = 0.01;
constant float c = 0.1;

inline bool
physicsField_applies(
                     PhysicsFieldStruct physicsField,
                     RigidBodyStruct rigidBody)
{
    float3 halfExtent = physicsField.halfExtent;
    
    if (halfExtent.x < 0 || halfExtent.y < 0 || halfExtent.z < 0) return false;

    bool3 all = (rigidBody.centerOfMass >= physicsField.position - halfExtent && rigidBody.centerOfMass <= physicsField.position + halfExtent);
    return all.x == true && all.y == true && all.z == true;
}

inline RigidBodyStruct
rigidBody_applyForce(
                     RigidBodyStruct rigidBody,
                     float3 force,
                     float3 torque)
{
    rigidBody.force += force;
    rigidBody.torque += torque;

    // FIXME (if parentId != nil ... then             torque += cross(rotation.act(-localPivot), force)
    
    return rigidBody;
}

kernel void
applyPhysicsFields(
                   constant PhysicsFieldStruct & physicsField [[ buffer(BufferIndexPhysicsField) ]],
                   device RigidBodyStruct * rigidBodies [[ buffer(BufferIndexRigidBodies) ]],
                   uint gid [[ thread_position_in_grid ]])
{
    RigidBodyStruct rigidBody = rigidBodies[gid];
    
    if (physicsField_applies(physicsField, rigidBody)) {
        float3 delta = physicsField.position - rigidBody.centerOfMass;
        float distance = length(delta);
        if (distance > 0) {
            float3 direction = normalize(delta);

            // NOTE: this is a bell-shaped curve, so objects very far and very near are weakly affected
            float3 force = direction * a * pow(M_E_F, -sqr(distance - b)/(2*c));
            rigidBody = rigidBody_applyForce(rigidBody, force, 0.5);
            rigidBodies[gid] = rigidBody;
        }
    }
}
