#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"

using namespace metal;

constant int rangeCount [[ function_constant(FunctionConstantIndexRangeCount) ]];

struct UpdateRigidBodiesIn {
    device ushort *parentId;
    device packed_half3 *localPivot;
    device packed_float3 *localInertiaTensor;
    device packed_half3 *theta;
    device quath *localJointRotation;
    device packed_half3 *localJointPosition;
};

struct UpdateRigidBodiesOut {
    device packed_half3 *pivot;
    device packed_half3 *centerOfMass;
    device quath *rotation;
    device quath *jointRotation;
    device InertiaTensor *inertiaTensor;
};

kernel void
updateRigidBodies(
                  device ushort   *in_parentId,
                  device packed_half3   *in_localPivot,
                  device packed_float3 *in_localInertiaTensor,
                  device packed_half3 *in_localJointPosition,
                  device quath    *in_localJointRotation,
                  device packed_half3 *in_theta,

                  device packed_half3   *out_pivot,
                  device packed_half3   *out_centerOfMass,
                  device InertiaTensor *out_inertiaTensor,
                  device quath    *out_rotation,
                  device quath    *out_jointRotation,

                  constant ushort * deltas,
                  uint gid [[ thread_position_in_grid ]])
{
    UpdateRigidBodiesIn in = {
        .parentId = in_parentId,
        .localPivot = in_localPivot,
        .localInertiaTensor = in_localInertiaTensor,
        .localJointPosition = in_localJointPosition,
        .localJointRotation = in_localJointRotation,
        .theta = in_theta,
    };
    UpdateRigidBodiesOut out = {
        .pivot = out_pivot,
        .centerOfMass = out_centerOfMass,
        .inertiaTensor = out_inertiaTensor,
        .rotation = out_rotation,
        .jointRotation = out_jointRotation,
    };

    uint previousLowerBound = 0;
    uint lowerBound = deltas[0];
    for (int i = 1; i < rangeCount; i++) {
        ushort delta = deltas[i];

        if (gid < delta) {
            const int id = lowerBound + gid;
            const ushort parentId = previousLowerBound + in.parentId[id];
            const float3 parentPivot = (float3)out.pivot[parentId];
            const quatf parentRotation = (quatf)out.rotation[parentId];

            const quatf localJointRotation = (quatf)in.localJointRotation[id];
            const float3 localJointPosition = (float3)in.localJointPosition[id];

            const packed_float3 localInertiaTensor = in.localInertiaTensor[id];
            const float3 localPivot = (float3)in.localPivot[id];
            const float3 theta = (float3)in.theta[id];

            const quatf jointRotation = quat_multiply(parentRotation, localJointRotation);
            float3 pivot = parentPivot + quat_act(parentRotation, localJointPosition);

            const quatf localRotation = length(theta) < 10e-10 ? quatf(0,0,0,1) : quat_from_axis_angle(normalize(theta), length(theta));
            const quatf rotation = normalize(quat_multiply(jointRotation, localRotation));

            const float3x3 R = float3x3_from_quat(rotation);

            float3x3 IRt = R;
            IRt[0] *= localInertiaTensor.x;
            IRt[1] *= localInertiaTensor.y;
            IRt[2] *= localInertiaTensor.z;
            IRt = transpose(IRt);
            const float3x3 inertiaTensor = R * IRt; // R * InertiaTensor * R^T
            const float3 centerOfMass = pivot + quat_act(rotation, -localPivot);

            out.pivot[id] = (packed_half3)pivot;
            out.centerOfMass[id] = (packed_half3)centerOfMass;
            out.inertiaTensor[id] = inertiaTensor_from_float3x3(inertiaTensor);
            out.rotation[id] = (quath)rotation;
            out.jointRotation[id] = (quath)jointRotation;
        }
        previousLowerBound = lowerBound;
        lowerBound += delta;
        threadgroup_barrier(mem_flags::mem_device);
    }
}

