#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
#import "Debug.metal"
using namespace metal;

constant int rangeCount [[ function_constant(FunctionConstantIndexRangeCount) ]];

typedef struct {
    device float *mass;
    device float3 *force;
    device float3 *torque;
    device float3 *centerOfMass;
    device float3x3 *inertiaTensor;
} UpdateCompositeBodiesOut;

typedef struct {
    device int2 *childInfo;
    device ushort *climberCount;
    device int *firstClimberId;
    device float *mass;
    device float3 *pivot;
    device float3 *force;
    device float3 *torque;
    device float3 *centerOfMass;
    device float3x3 *inertiaTensor;
    UpdateCompositeBodiesOut children;
} UpdateCompositeBodiesIn;

inline void
climb(
      int id,
      float m, float3 f, float3 pivot, float3 t, float3 c, float3x3 it,
      const UpdateCompositeBodiesIn in,
      const UpdateCompositeBodiesOut out)
{
                    ushort climberCount = in.climberCount[id];
                    if (climberCount == 0) return;
    //                int firstClimberId = in.firstClimberId[id];

                    float mass;
                    float3 force, torque, centerOfMass;
                    float3x3 inertiaTensor;
                    int climberId;
                    for (ushort i = 0; i < climberCount; i++) {
                        climberId = id + 29523 + i;

                        mass = m + in.mass[climberId];
                        force = f + in.force[climberId];
                        torque = cross(in.pivot[climberId] - pivot, f) + t + in.torque[climberId];
                        centerOfMass = m * c + in.mass[climberId] * in.centerOfMass[climberId];
                        centerOfMass /= mass;

                        float3x3 inertiaTensor = in.inertiaTensor[climberId] - in.mass[climberId] * sqr(crossMatrix(in.centerOfMass[climberId] - centerOfMass));

                        inertiaTensor += it - m * sqr(crossMatrix(c - centerOfMass));

                        m = mass;
                        f = force;
                        t = torque;
                        c = centerOfMass;
                        it = inertiaTensor;
                        pivot = in.pivot[climberId];
                    }

                    out.mass[climberId] = mass;
                    out.force[climberId] = force;
                    out.torque[climberId] = torque;
                    out.centerOfMass[climberId] = centerOfMass;
                    out.inertiaTensor[climberId] = inertiaTensor;
}

inline void
update(
       const int id,
       const UpdateCompositeBodiesIn in,
       const UpdateCompositeBodiesOut out)
{
    int2 childInfo = in.childInfo[id];

    float mass = in.mass[id];
    float3 force = in.force[id];
    float3 pivot = in.pivot[id];
    float3 torque = in.torque[id];
    float3 centerOfMass = mass * in.centerOfMass[id];

    int firstChildId = childInfo.y;
    for (ushort i = 0; i < childInfo.x; i++) {
//        int childId = firstChildId + i;
        int childId = id + 29523 + i;

        mass += in.children.mass[childId];
        force += in.children.force[childId];
        torque += cross(in.pivot[childId] - pivot, in.children.force[childId]) + in.children.torque[childId];
        centerOfMass += in.children.mass[childId] * in.children.centerOfMass[childId];
    }
    centerOfMass /= mass;

    float3x3 inertiaTensor = in.inertiaTensor[id] - in.mass[id] * sqr(crossMatrix(in.centerOfMass[id] - centerOfMass));

    for (ushort i = 0; i < childInfo.x; i++) {
        int childId = firstChildId + i;
        inertiaTensor += in.children.inertiaTensor[childId] - in.children.mass[childId] * sqr(crossMatrix(in.children.centerOfMass[childId] - centerOfMass));
    }

    out.mass[id] = mass;
    out.force[id] = force;
    out.torque[id] = torque;
    out.centerOfMass[id] = centerOfMass;
    out.inertiaTensor[id] = inertiaTensor;

    climb(id, mass, force, pivot, torque, centerOfMass, inertiaTensor, in, out);
}

kernel void
updateCompositeBodies(
                      device int2 *in_childInfo,
                      device ushort *in_climberCount,
                      device int *in_firstClimberId,
                      device float *in_mass,
                      device float3 *in_pivot,
                      device float3 *in_force,
                      device float3 *in_torque,
                      device float3 *in_centerOfMass,
                      device float3x3 *in_inertiaTensor,

                      device float *out_mass,
                      device float3 *out_force,
                      device float3 *out_torque,
                      device float3 *out_centerOfMass,
                      device float3x3 *out_inertiaTensor,

                      constant int2 * ranges,
                      uint gid [[ thread_position_in_grid ]])
{
    UpdateCompositeBodiesOut out = {
        .mass = out_mass,
        .force = out_force,
        .torque = out_torque,
        .centerOfMass = out_centerOfMass,
        .inertiaTensor = out_inertiaTensor
    };

    for (ushort i = 0; i < rangeCount; i++) {
        int2 range = ranges[i];
        int lowerBound = range.x;
        int upperBound = range.y;
        if ((int)gid < upperBound - lowerBound) {
            int id = lowerBound + gid;

            if (i == 0) {
                UpdateCompositeBodiesIn in = {
                    .childInfo = in_childInfo,
                    .climberCount = in_climberCount,
                    .firstClimberId = in_firstClimberId,
                    .mass = in_mass,
                    .pivot = in_pivot,
                    .force = in_force,
                    .torque = in_torque,
                    .centerOfMass = in_centerOfMass,
                    .inertiaTensor = in_inertiaTensor
                };

                float m = in.mass[id];
                float3 f = in.force[id];
                float3 pivot = in.pivot[id];
                float3 t = in.torque[id];
                float3 c = in.centerOfMass[id];
                float3x3 it = in.inertiaTensor[id];

                climb(id, m, f, pivot, t, c, it, in, out);
            } else if (i == 1) {
                UpdateCompositeBodiesIn in = {
                    .childInfo = in_childInfo,
                    .climberCount = in_climberCount,
                    .firstClimberId = in_firstClimberId,
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
                update(id, in, out);
            } else {
                UpdateCompositeBodiesIn in = {
                    .childInfo = in_childInfo,
                    .climberCount = in_climberCount,
                    .firstClimberId = in_firstClimberId,
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
                update(id, in, out);
            }
        }
        threadgroup_barrier(mem_flags::mem_device);
    }
}
