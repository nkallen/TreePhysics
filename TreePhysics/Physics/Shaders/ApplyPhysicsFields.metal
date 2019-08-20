#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"

using namespace metal;

constant half a = 0.05;
constant half b = 0.01;
constant half c = 0.1;

inline bool
physicsField_applies(
                     PhysicsFieldStruct physicsField,
                     RigidBodyStruct rigidBody)
{
    half3 halfExtent = physicsField.halfExtent;
    
    if (halfExtent.x < 0 || halfExtent.y < 0 || halfExtent.z < 0) return false;
    
    bool3 all = (rigidBody.position >= physicsField.position - halfExtent && rigidBody.position <= physicsField.position + halfExtent);
    return all.x == true && all.y == true && all.z == true;
}

inline half3
rigidBody_convert(
                  RigidBodyStruct rigidBody,
                  half3 position)
{
    return rigidBody.position + rigidBody.rotation * position;
}

// NOTE: location is along the Y axis of the cylinder/branch, relative to the pivot/parent's end
// distance is in normalize [0..1] coordinates
inline RigidBodyStruct
rigidBody_applyForce(
                     RigidBodyStruct rigidBody,
                     half3 force,
                     half distance)
{
    if (distance < 0 || distance > 1) return rigidBody;
    
    rigidBody.force += force;
    rigidBody.torque += cross(rigidBody_convert(rigidBody, half3(0, 1, 0) * distance * rigidBody.length) - rigidBody.position, force);
    
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
        half3 delta = physicsField.position - rigidBody.position;
        half distance = length(delta);
        if (distance > 0) {
            half3 direction = normalize(delta);
            // NOTE: this is a bell-shaped curve, so objects very far and very near are weakly affected
            half3 force = direction * a * pow(M_E_H, -sqr(distance - b)/(2*c));

            rigidBody = rigidBody_applyForce(rigidBody, force, 0.5);

            rigidBodies[gid] = rigidBody;
        }
    }
}
