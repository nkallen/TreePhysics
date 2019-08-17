#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
using namespace metal;

constant int rangeCount [[ function_constant(FunctionConstantIndexRangeCount) ]];

inline float3 joint_position(
                             JointStruct joint,
                             RigidBodyStruct parentRigidBody)
{
    return parentRigidBody.position + parentRigidBody.rotation * float3(0, parentRigidBody.length, 0);
}

inline float3x3 joint_localRotation(
                                    JointStruct joint)
{
    return matrix_rotate(joint.θ[0]);
}

inline float3x3 rigidBody_localInertiaTensor(
                                             RigidBodyStruct rigidBody)
{
    float mass = rigidBody.mass;
    float length = rigidBody.length;
    float radius = rigidBody.radius;

    float momentOfInertiaAboutY = 1.0/12 * mass * length * length; // Moment of Inertia of a rod about its center of mass;
    float momentOfInertiaAboutX = 1.0/4 * mass * radius * radius; // MoI of a disc about its center
    float momentOfInertiaAboutZ = 1.0/4 * mass * radius * radius; // ditto

    // Inertia tensor of a rod about its center of mass, see http://scienceworld.wolfram.com/physics/MomentofInertiaCylinder.html
    // and https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    return float3x3(momentOfInertiaAboutY + momentOfInertiaAboutX,
                    momentOfInertiaAboutZ + momentOfInertiaAboutX,
                    momentOfInertiaAboutX + momentOfInertiaAboutY);
}

inline float3 rigidBody_localCenterOfMass(
                                          RigidBodyStruct rigidBody)
{
    return float3(0, 1, 0) * rigidBody.length / 2;
}

inline RigidBodyStruct
updateRigidBody(
                const RigidBodyStruct parentRigidBody,
                const JointStruct parentJoint,
                RigidBodyStruct rigidBody)
{
    float3x3 parentJointLocalRotation = joint_localRotation(parentJoint);
    float3x3 parentJointRotation = parentRigidBody.rotation * parentJointLocalRotation;
    float3 parentJointPosition = joint_position(parentJoint, parentRigidBody);

    rigidBody.rotation = parentJointRotation * rigidBody.localRotation;
    rigidBody.position = parentJointPosition;

    rigidBody.inertiaTensor = rigidBody.rotation * rigidBody_localInertiaTensor(rigidBody) * transpose(rigidBody.rotation);
    rigidBody.centerOfMass = rigidBody.position + rigidBody.rotation * rigidBody_localCenterOfMass(rigidBody);

    return rigidBody;
}

inline RigidBodyStruct
rigidBody_climbDown(
                    const RigidBodyStruct rigidBody,
                    device RigidBodyStruct * rigidBodies,
                    device JointStruct * joints)
{
    RigidBodyStruct parentRigidBody, currentRigidBody;
    for (short i = rigidBody.climberCount - 1; i >= 0; i--) {
        int id = rigidBody.climberOffset + i;

        RigidBodyStruct next = rigidBodies[id];
        JointStruct parentJoint = joints[id];
        if (i == rigidBody.climberCount - 1) {
            parentRigidBody = rigidBodies[next.parentId];
        } else {
            parentRigidBody = currentRigidBody;
        }
        currentRigidBody = next;

        currentRigidBody = updateRigidBody(parentRigidBody, parentJoint, currentRigidBody);
        rigidBodies[id] = currentRigidBody;
    }
    return currentRigidBody;
}

kernel void
updateRigidBodies(
                  device RigidBodyStruct * rigidBodies [[ buffer(BufferIndexRigidBodies) ]],
                  device JointStruct * joints [[ buffer(BufferIndexJoints) ]],
                  constant int2 * ranges [[ buffer(BufferIndexRanges) ]],
                  uint gid [[ thread_position_in_grid ]])
{
    for (int i = 0; i < rangeCount; i++) {
        int2 range = ranges[i];
        int lowerBound = range.x;
        int upperBound = range.y;
        if ((int)gid < upperBound - lowerBound) {
            int id = lowerBound + gid;

            RigidBodyStruct rigidBody = rigidBodies[id];
            if (rigidBody.parentId != -1) {
                RigidBodyStruct parentRigidBody;
                if (rigidBody.climberCount > 0) {
                    parentRigidBody = rigidBody_climbDown(rigidBody, rigidBodies, joints);
                } else {
                    parentRigidBody = rigidBodies[rigidBody.parentId];
                }
                // potentially can optimize away:
                const JointStruct parentJoint = joints[id];

                rigidBody = updateRigidBody(parentRigidBody, parentJoint, rigidBody);
                rigidBodies[id] = rigidBody;
            }
        }
        threadgroup_barrier(mem_flags::mem_device);
    }
}

