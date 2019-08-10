#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
#import "Debug.metal"
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
                RigidBodyStruct rigidBody,
                FUNCTION_DEBUG_FORMAL_PARAMETERS
                )
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
                    RigidBodyStruct parentRigidBody,
                    RigidBodyStruct climbers[10],
                    JointStruct joints[10],
                    device RigidBodyStruct * rigidBodies,
                    FUNCTION_DEBUG_FORMAL_PARAMETERS
                    )
{
    RigidBodyStruct currentRigidBody;

    for (short i = rigidBody.climberCount - 1; i >= 0; i--) {
        currentRigidBody = climbers[i];
        JointStruct parentJoint = joints[i];

        int id = parentRigidBody.childIds[0];

        currentRigidBody = updateRigidBody(parentRigidBody, parentJoint, currentRigidBody, DEBUG_PARAMETERS);
        rigidBodies[id] = currentRigidBody;
        parentRigidBody = currentRigidBody;
    }
    return parentRigidBody;
}

inline RigidBodyStruct rigidBody_climbers(
                                          const RigidBodyStruct rigidBody,
                                          device RigidBodyStruct * rigidBodies,
                                          device JointStruct * joints,
                                          RigidBodyStruct climbers[10],
                                          JointStruct climberJoints[10],
                                          FUNCTION_DEBUG_FORMAL_PARAMETERS
                                          )
{
    for (ushort i = 0; i < 10; i++) {
        int climberId = rigidBody.climberIds[i];
        RigidBodyStruct climber = rigidBodies[climberId];
        climbers[i] = climber;
        climberJoints[i] = joints[climberId];
    }

    return rigidBodies[climbers[rigidBody.climberCount - 1].parentId];
}

kernel void
updateRigidBodies(
                  device RigidBodyStruct * rigidBodies [[ buffer(BufferIndexRigidBodies) ]],
                  device JointStruct * joints [[ buffer(BufferIndexJoints) ]],
                  constant int2 * ranges [[ buffer(BufferIndexRanges) ]],
                  uint gid [[ thread_position_in_grid ]],
                  KERNEL_DEBUG_FORMAL_PARAMETERS
                  )
{
    for (int i = 0; i < rangeCount; i++) {
        int2 range = ranges[i];
        int lowerBound = range.x;
        int upperBound = range.y;
        if ((int)gid >= lowerBound && (int)gid < upperBound) {
            int id = lowerBound + gid;

            RigidBodyStruct rigidBody = rigidBodies[id];
            if (rigidBody.parentId != -1) {
                const JointStruct parentJoint = joints[0];
                RigidBodyStruct parentRigidBody;
                if (rigidBody.climberCount > 0) {

                    RigidBodyStruct climbers[10];
                    JointStruct climberJoints[10];
                    parentRigidBody = rigidBody_climbers(rigidBody, rigidBodies, joints, climbers, climberJoints, DEBUG_PARAMETERS);
                    parentRigidBody = rigidBody_climbDown(rigidBody, parentRigidBody, climbers, climberJoints, rigidBodies, DEBUG_PARAMETERS);
                } else {
                    parentRigidBody = rigidBodies[rigidBody.parentId];
                }
                rigidBody = updateRigidBody(parentRigidBody, parentJoint, rigidBody, DEBUG_PARAMETERS);
                rigidBodies[id] = rigidBody;
            }
        }
        threadgroup_barrier(mem_flags::mem_threadgroup);
    }
}
