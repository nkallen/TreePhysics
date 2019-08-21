#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
#import "Print.metal"

using namespace metal;

constant int rangeCount [[ function_constant(FunctionConstantIndexRangeCount) ]];

inline half3 joint_position(
                             JointStruct joint,
                             RigidBodyStruct parentRigidBody)
{
    return parentRigidBody.position + parentRigidBody.rotation * half3(0, parentRigidBody.length, 0);
}

inline half3x3 joint_localRotation(
                                    JointStruct joint)
{
    return matrix_rotate(joint.θ[0]);
}

inline float3x3 rigidBody_localInertiaTensor(
                                             RigidBodyStruct rigidBody)
{
    half mass = rigidBody.mass;
    half length = rigidBody.length;
    half radius = rigidBody.radius;

    float momentOfInertiaAboutY = 1.0/12 * mass * length * length; // Moment of Inertia of a rod about its center of mass;
    float momentOfInertiaAboutX = 1.0/4 * mass * radius * radius; // MoI of a disc about its center
    float momentOfInertiaAboutZ = 1.0/4 * mass * radius * radius; // ditto

    // Inertia tensor of a rod about its center of mass, see http://scienceworld.wolfram.com/physics/MomentofInertiaCylinder.html
    // and https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    return float3x3(momentOfInertiaAboutY + momentOfInertiaAboutX, 0, 0,
                    0, momentOfInertiaAboutZ + momentOfInertiaAboutX, 0,
                    0, 0, momentOfInertiaAboutX + momentOfInertiaAboutY);
}

inline half3 rigidBody_localCenterOfMass(
                                          RigidBodyStruct rigidBody)
{
    return half3(0, 1, 0) * rigidBody.length / 2;
}

inline RigidBodyStruct
updateRigidBody(
                const RigidBodyStruct parentRigidBody,
                const JointStruct parentJoint,
                RigidBodyStruct rigidBody,
                thread Debug & debug)
{
//    debug << "updating rigid body\n";
    half3x3 parentJointLocalRotation = joint_localRotation(parentJoint);
    half3x3 parentJointRotation = parentRigidBody.rotation * parentJointLocalRotation;
    half3 parentJointPosition = joint_position(parentJoint, parentRigidBody);

//    debug << "joint.θ[0]=" << parentJoint.θ[0] << "\n";
//    debug << "parentJointLocalRotation=" << parentJointLocalRotation << "\n";
//    debug << "parentJointRotation=" << parentRigidBody.rotation << " * " << parentJointLocalRotation << " = " << parentJointRotation << "\n";

    rigidBody.rotation = parentJointRotation * rigidBody.localRotation;
    rigidBody.position = parentJointPosition;

//    debug << "rotation=" << rigidBody.rotation << "\n";
//    debug << "position=" << rigidBody.position << "\n";

    rigidBody.inertiaTensor = (float3x3)rigidBody.rotation * rigidBody_localInertiaTensor(rigidBody) * (float3x3)transpose(rigidBody.rotation);
    rigidBody.centerOfMass = rigidBody.position + rigidBody.rotation * rigidBody_localCenterOfMass(rigidBody);

    return rigidBody;
}

inline RigidBodyStruct
rigidBody_climbDown(
                    const RigidBodyStruct rigidBody,
                    device RigidBodyStruct * rigidBodies,
                    device JointStruct * joints,
                    thread Debug & debug)
{
    RigidBodyStruct parentRigidBody, currentRigidBody;
    for (short i = rigidBody.climberCount - 1; i >= 0; i--) {
        int id = rigidBody.climberOffset + i;
//        debug << "climbing down id: " << id << "\n";

        RigidBodyStruct next = rigidBodies[id];
        JointStruct parentJoint = joints[id];
        if (i == rigidBody.climberCount - 1) {
            parentRigidBody = rigidBodies[next.parentId];
        } else {
            parentRigidBody = currentRigidBody;
        }
        currentRigidBody = next;

        currentRigidBody = updateRigidBody(parentRigidBody, parentJoint, currentRigidBody, debug);
        rigidBodies[id] = currentRigidBody;
    }
    return currentRigidBody;
}

kernel void
updateRigidBodies(
                  device RigidBodyStruct * rigidBodies [[ buffer(BufferIndexRigidBodies) ]],
                  device JointStruct * joints [[ buffer(BufferIndexJoints) ]],
                  constant int2 * ranges [[ buffer(BufferIndexRanges) ]],
                  uint gid [[ thread_position_in_grid ]],
                  device char * buf [[ buffer(BufferIndexDebugString) ]]
                  )
{
    Debug debug = Debug(buf + gid*8192, 8192);

    for (int i = 0; i < rangeCount; i++) {
        int2 range = ranges[i];
        int lowerBound = range.x;
        int upperBound = range.y;
        if ((int)gid < upperBound - lowerBound) {
            int id = lowerBound + gid;

//            debug << "id: " << id << "\n";
            RigidBodyStruct rigidBody = rigidBodies[id];
            if (rigidBody.parentId != -1) { // FIXME shouldn't be necessary anymore?
                RigidBodyStruct parentRigidBody;
                if (rigidBody.climberCount > 0) {
//                    debug << "has climbers: " << rigidBody.climberCount << "\n";
                    parentRigidBody = rigidBody_climbDown(rigidBody, rigidBodies, joints, debug);
                } else {
//                    debug << "no climbers: " << "\n";
                    parentRigidBody = rigidBodies[rigidBody.parentId];
                }
                // potentially can optimize away:
                const JointStruct parentJoint = joints[id];

                rigidBody = updateRigidBody(parentRigidBody, parentJoint, rigidBody, debug);
                rigidBodies[id] = rigidBody;
            }
        }
        threadgroup_barrier(mem_flags::mem_device);
    }
}

