#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"

using namespace metal;

constant int rangeCount [[ function_constant(FunctionConstantIndexRangeCount) ]];

inline float3x3 rigidBody_localRotation(
                                    JointStruct joint)
{
    return matrix_rotate(joint.θ[0]);
}

inline RigidBodyStruct
updateRigidBody(
                const RigidBodyStruct parentRigidBody,
                const JointStruct parentJoint,
                RigidBodyStruct rigidBody)
{
    float3 parentJointPosition = rigidBody.pivot;

    rigidBody.rotation = parentRigidBody.rotation * rigidBody.jointLocalRotation * rigidBody_localRotation(parentJoint);
    rigidBody.pivot = parentJointPosition;

    rigidBody.inertiaTensor = (float3x3)rigidBody.rotation * rigidBody.localInertiaTensor * (float3x3)transpose(rigidBody.rotation);
    rigidBody.centerOfMass = rigidBody.pivot + rigidBody.rotation * (-rigidBody.localPivot);

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
        threadgroup_barrier(mem_flags::mem_device);
    }
}

