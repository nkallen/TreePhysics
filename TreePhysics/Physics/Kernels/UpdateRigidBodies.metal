#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"

using namespace metal;

constant int rangeCount [[ function_constant(FunctionConstantIndexRangeCount) ]];

struct UpdateRigidBodiesIn {
    device uint *parentId;
    device packed_half3 *localPivot;
    device packed_float3 *localInertiaTensor;
    device packed_half3 *theta;
    device quath *localJointOrientation;
    device packed_half3 *localJointPosition;
};

struct UpdateRigidBodiesOut {
    device packed_half3 *pivot;
    device packed_half3 *centerOfMass;
    device quath *orientation;
    device quath *jointOrientation;
    device InertiaTensor *inertiaTensor;
    device packed_half3 *velocity;
    device packed_half3 *angularVelocity;
    device packed_half3 *acceleration;
    device packed_half3 *angularAcceleration;
};

kernel void
updateRigidBodies(
                  device uint        *in_parentId,
                  device packed_half3  *in_localPivot,
                  device packed_float3 *in_localInertiaTensor,
                  device packed_half3  *in_localJointPosition,
                  device quath         *in_localJointOrientation,
                  device packed_half3  *in_theta,

                  device packed_half3  *out_pivot,
                  device packed_half3  *out_centerOfMass,
                  device InertiaTensor *out_inertiaTensor,
                  device quath         *out_orientation,
                  device quath         *out_jointOrientation,
                  device packed_half3  *out_velocity,
                  device packed_half3  *out_angularVelocity,
                  device packed_half3  *out_acceleration,
                  device packed_half3  *out_angularAcceleration,

                  constant ushort * deltas,
                  uint gid [[ thread_position_in_grid ]])
{
    UpdateRigidBodiesIn in = {
        .parentId = in_parentId,
        .localPivot = in_localPivot,
        .localInertiaTensor = in_localInertiaTensor,
        .localJointPosition = in_localJointPosition,
        .localJointOrientation = in_localJointOrientation,
        .theta = in_theta,
    };
    UpdateRigidBodiesOut out = {
        .pivot = out_pivot,
        .centerOfMass = out_centerOfMass,
        .inertiaTensor = out_inertiaTensor,
        .orientation = out_orientation,
        .jointOrientation = out_jointOrientation,
        .velocity = out_velocity,
        .angularVelocity = out_angularVelocity,
        .acceleration = out_acceleration,
        .angularAcceleration = out_angularAcceleration,
    };

    uint previousLowerBound = 0;
    uint lowerBound = deltas[0];
    for (int i = 0; i < rangeCount; i++) {
        ushort delta = deltas[i];

        if (gid < delta) {
            const int id = lowerBound + gid;
            const uint parentId = in.parentId[id];
            if (parentId != NO_PARENT) {
                const float3 parentPivot = (float3)out.pivot[parentId];
                const quatf parentOrientation = (quatf)out.orientation[parentId];

                const quatf localJointOrientation = (quatf)in.localJointOrientation[id];
                const float3 localJointPosition = (float3)in.localJointPosition[id];

                const packed_float3 localInertiaTensor = in.localInertiaTensor[id];
                const float3 localPivot = (float3)in.localPivot[id];
                const float3 angle = (float3)in.theta[id*3+0];

                const quatf jointOrientation = quat_multiply(parentOrientation, localJointOrientation);
                float3 pivot = parentPivot + quat_act(parentOrientation, localJointPosition);

                const quatf localOrientation = length(angle) < 10e-10 ? quatf(0,0,0,1) : quat_from_axis_angle(normalize(angle), length(angle));
                const quatf orientation = normalize(quat_multiply(jointOrientation, localOrientation));

                const float3x3 R = float3x3_from_quat(orientation);

                float3x3 IRt = R;
                IRt[0] *= localInertiaTensor.x;
                IRt[1] *= localInertiaTensor.y;
                IRt[2] *= localInertiaTensor.z;
                IRt = transpose(IRt);
                const float3x3 inertiaTensor = R * IRt; // R * InertiaTensor * R^T
                const float3 centerOfMass = pivot + quat_act(orientation, -localPivot);

                const float3 parentAngularVelocity = (float3)out.angularVelocity[parentId];
                const float3 parentAngularAcceleration = (float3)out.angularAcceleration[parentId];
                const float3 angularVelocity = parentAngularVelocity + quat_act(jointOrientation, (float3)in.theta[parentId*3+1]);
                const float3 angularAcceleration = (float3)out.angularAcceleration[parentId] + quat_act(localJointOrientation, (float3)in.theta[id*3+2]) + skew(parentAngularVelocity) * angularVelocity;

                float3 velocity = (float3)out.velocity[parentId];
                velocity += skew(parentAngularVelocity) * quat_act(parentOrientation, localJointPosition);
                velocity -= skew(angularVelocity) * quat_act(orientation, localPivot);
                float3 acceleration = (float3)out.acceleration[parentId] + (skew(parentAngularAcceleration) + sqr(skew(parentAngularVelocity))) * quat_act(parentOrientation, localJointPosition);
                acceleration -= (skew(angularAcceleration) + sqr(skew(angularVelocity))) * quat_act(orientation, localPivot);

                out.pivot[id] = (packed_half3)pivot;
                out.centerOfMass[id] = (packed_half3)centerOfMass;
                out.inertiaTensor[id] = inertiaTensor_from_float3x3(inertiaTensor);
                out.orientation[id] = (quath)orientation;
                out.jointOrientation[id] = (quath)jointOrientation;
                out.velocity[id] = (packed_half3)velocity;
                out.angularVelocity[id] = (packed_half3)angularVelocity;
                out.acceleration[id] = (packed_half3)acceleration;
                out.angularAcceleration[id] = (packed_half3)angularAcceleration;
            }
        }
        previousLowerBound = lowerBound;
        lowerBound += delta;
        threadgroup_barrier(mem_flags::mem_device);
    }
}

