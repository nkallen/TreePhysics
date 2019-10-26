#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
#import "Debug.metal"
using namespace metal;

constant int rangeCount [[ function_constant(FunctionConstantIndexRangeCount) ]];

typedef struct {
    device float *mass;
    device vector_float3 *force;
    device vector_float3 *torque;
    device vector_float3 *centerOfMass;
    device matrix_float3x3 *inertiaTensor;
} UpdateCompositeBodiesOut;

typedef struct {
    device ushort *childCount;
    device int *firstChildId;
    device float *mass;
    device vector_float3 *pivot;
    device vector_float3 *force;
    device vector_float3 *torque;
    device vector_float3 *centerOfMass;
    device matrix_float3x3 *inertiaTensor;
    UpdateCompositeBodiesOut children;
} UpdateCompositeBodiesIn;

inline void
update(
       int id,
       UpdateCompositeBodiesIn in,
       UpdateCompositeBodiesOut out)
{
    ushort childCount = in.childCount[id];
    int firstChildId = in.firstChildId[id];
    float mass = in.mass[id];
    float3 force = in.force[id];
    float3 pivot = in.pivot[id];
    float3 torque = in.torque[id];
    float3 centerOfMass = mass * in.centerOfMass[id];

    for (ushort i = 0; i < childCount; i++) {
        int childId = firstChildId + i;
        mass += in.children.mass[childId];
        force += in.children.force[childId];
        torque += cross(in.pivot[childId] - pivot, in.children.force[childId]) + in.children.torque[childId];
        centerOfMass += in.children.mass[childId] * in.children.centerOfMass[childId];
    }
    centerOfMass /= mass;

    float3x3 inertiaTensor = in.inertiaTensor[id] - in.mass[id] * sqr(crossMatrix(in.centerOfMass[id] - centerOfMass));

    for (ushort i = 0; i < childCount; i++) {
        int childId = firstChildId + i;
        inertiaTensor += in.children.inertiaTensor[childId] - in.children.mass[childId] * sqr(crossMatrix(in.children.centerOfMass[childId] - centerOfMass));
    }

    out.mass[id] = mass;
    out.force[id] = force;
    out.torque[id] = torque;
    out.centerOfMass[id] = centerOfMass;
    out.inertiaTensor[id] = inertiaTensor;
}

kernel void
updateCompositeBodies(
                      device ushort *in_childCount,
                      device int *in_firstChildId,
                      device float *in_mass,
                      device vector_float3 *in_pivot,
                      device vector_float3 *in_force,
                      device vector_float3 *in_torque,
                      device vector_float3 *in_centerOfMass,
                      device matrix_float3x3 *in_inertiaTensor,

                      device float *out_mass,
                      device vector_float3 *out_force,
                      device vector_float3 *out_torque,
                      device vector_float3 *out_centerOfMass,
                      device matrix_float3x3 *out_inertiaTensor,

                      constant int2 * ranges,
                      uint gid [[ thread_position_in_grid ]])
{
    int offset = -1;
    for (ushort i = 0; i < rangeCount; i++) {
        int2 range = ranges[i];
        int lowerBound = range.x;
        int upperBound = range.y;
        if ((int)gid < upperBound - lowerBound) {
            int id = lowerBound + gid;

            if (i == 0) {
            } else if (i == 1) {
                UpdateCompositeBodiesIn in = {
                    .childCount = in_childCount,
                    .firstChildId = in_firstChildId,
                    .mass = in_mass,
                    .pivot = in_pivot,
                    .force = in_force,
                    .torque = in_torque,
                    .centerOfMass = in_centerOfMass,
                    .inertiaTensor = in_inertiaTensor,
                    .children = {
                        .mass = in_mass,
                        .force = in_force,
                        .torque = in_torque,
                        .centerOfMass = in_centerOfMass,
                        .inertiaTensor = in_inertiaTensor
                    }
                };
                UpdateCompositeBodiesOut out = {
                    .mass = out_mass,
                    .force = out_force,
                    .torque = out_torque,
                    .centerOfMass = out_centerOfMass,
                    .inertiaTensor = out_inertiaTensor
                };
                update(id, in, out);
            } else {
                UpdateCompositeBodiesIn in = {
                    .childCount = in_childCount,
                    .firstChildId = in_firstChildId,
                    .mass = in_mass,
                    .pivot = in_pivot,
                    .force = in_force,
                    .torque = in_torque,
                    .centerOfMass = in_centerOfMass,
                    .inertiaTensor = in_inertiaTensor,
                    .children = {
                        .mass = out_mass,
                        .force = out_force,
                        .torque = out_torque,
                        .centerOfMass = out_centerOfMass,
                        .inertiaTensor = out_inertiaTensor
                    }
                };
                UpdateCompositeBodiesOut out = {
                    .mass = out_mass,
                    .force = out_force,
                    .torque = out_torque,
                    .centerOfMass = out_centerOfMass,
                    .inertiaTensor = out_inertiaTensor
                };
                update(id, in, out);
            }
            threadgroup_barrier(mem_flags::mem_device);
        }
        offset = lowerBound;
    }
}
