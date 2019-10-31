#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
#import "Debug.metal"
using namespace metal;

constant int rangeCount [[ function_constant(FunctionConstantIndexRangeCount) ]];

typedef struct {
    device half *mass;
    device packed_half3 *force;
    device packed_half3 *torque;
    device packed_half3 *centerOfMass;
    device half3x3 *inertiaTensor;
} UpdateCompositeBodiesOut;

typedef struct {
    device int2 *childInfo;
    device ushort *climberCount;
    device int *firstClimberId;
    device half *mass;
    device packed_half3 *pivot;
    device packed_half3 *force;
    device packed_half3 *torque;
    device packed_half3 *centerOfMass;
    device half3x3 *inertiaTensor;
    UpdateCompositeBodiesOut children;
} UpdateCompositeBodiesIn;

/*

inline void
climb(
      int id,
      half m, half3 f, half3 pivot, half3 t, half3 c, half3x3 it,
      const UpdateCompositeBodiesIn in,
      const UpdateCompositeBodiesOut out)
{
                    ushort climberCount = in.climberCount[id];
                    if (climberCount == 0) return;
    //                int firstClimberId = in.firstClimberId[id];

                    half mass;
                    half3 force, torque, centerOfMass;
                    half3x3 inertiaTensor;
                    int climberId;
                    for (ushort i = 0; i < climberCount; i++) {
                        climberId = id + 29523 + i;

                        mass = m + in.mass[climberId];
                        force = f + in.force[climberId];
                        torque = cross(in.pivot[climberId] - pivot, f) + t + in.torque[climberId];
                        centerOfMass = m * c + in.mass[climberId] * in.centerOfMass[climberId];
                        centerOfMass /= mass;

                        half3x3 inertiaTensor = in.inertiaTensor[climberId] - in.mass[climberId] * sqr(crossMatrix(in.centerOfMass[climberId] - centerOfMass));

                        inertiaTensor += it - m * sqr(crossMatrix(c - centerOfMass));

//                        out.mass[climberId] = mass;
//                        out.force[climberId] = force;
//                        out.torque[climberId] = torque;
//                        out.centerOfMass[climberId] = centerOfMass;
                        out.inertiaTensor[climberId] = half3x3(inertiaTensor);

                        m = mass;
                        f = force;
                        t = torque;
                        c = centerOfMass;
                        it = inertiaTensor;
                        pivot = in.pivot[climberId];
                    }
}

inline void
update(
       const int id,
       const UpdateCompositeBodiesIn in,
       const UpdateCompositeBodiesOut out)
{
    int2 childInfo = in.childInfo[id];

    half mass = in.mass[id];
    half3 force = in.force[id];
    half3 pivot = in.pivot[id];
    half3 torque = in.torque[id];
    half3 centerOfMass = mass * in.centerOfMass[id];

    int firstChildId = childInfo.y;
    for (ushort i = 0; i < childInfo.x; i++) {
//        int childId = firstChildId + i;
        int childId = id + 29523 + i;

        mass += in.children.mass[childId];
        force += packed_half3(in.children.force[childId]);
        torque += cross(in.pivot[childId] - pivot, in.children.force[childId]) + in.children.torque[childId];
        centerOfMass += in.children.mass[childId] * in.children.centerOfMass[childId];
    }
    centerOfMass /= mass;

    half3x3 inertiaTensor = in.inertiaTensor[id] - in.mass[id] * sqr(crossMatrix(in.centerOfMass[id] - centerOfMass));

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
 */

kernel void
updateCompositeBodies(
                      device int2 *in_childInfo,
                      device ushort *in_climberCount,
                      device int *in_firstClimberId,
                      device half *in_mass,
                      device packed_half3 *in_pivot,
                      device packed_half3 *in_force,
                      device packed_half3 *in_torque,
                      device packed_half3 *in_centerOfMass,
                      device half3x3 *in_inertiaTensor,

                      device half *out_mass,
                      device packed_half3 *out_force,
                      device packed_half3 *out_torque,
                      device packed_half3 *out_centerOfMass,
                      device half3x3 *out_inertiaTensor,

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

    int2 prev;
    for (ushort i = 0; i < rangeCount; i++) {
        int2 range = ranges[i];
        int lowerBound = range.x;
        int upperBound = range.y;
        if ((int)gid < upperBound - lowerBound) {
            int id = lowerBound + gid;
            int2 childInfo = in.childInfo[id];

            half mass = in.mass[id];
            half3 force = in.force[id];
            half3 torque = in.torque[id];
            half3 centerOfMass = in.centerOfMass[id];
            half3x3 inertiaTensor = in.inertiaTensor[id];

            for (ushort i = 0; i < (i > 0 ? 3 : 0); i++) {
                int childId = gid + prev.x + i * (prev.y - prev.x) / 3;
//                int childId = i + prev.x + gid * 3;
//                int childId = i + childInfo.y;

                mass += in.mass[childId];
                force += in.force[childId];
                torque += in.torque[childId];
                centerOfMass += in.centerOfMass[childId];
                inertiaTensor += in.inertiaTensor[childId];
            }

//            out.mass[id] = mass;
//            out.force[id] = (packed_half3)force;
//            out.torque[id] = (packed_half3)torque;
//            out.centerOfMass[id] = (packed_half3)centerOfMass;
            out.inertiaTensor[id] = (half3x3)inertiaTensor;

            ushort climberCount = in.climberCount[id];
            for (ushort i = 0; i < 6; i++) {
                int climberId = gid + 29523 + i * (206662-29523)/6;
//                int climberId = i + 29523 + gid * 6;
//                out.mass[climberId] = in.mass[climberId];
//                out.force[climberId] = (packed_half3)in.force[climberId];
//                out.torque[climberId] = (packed_half3)in.torque[climberId];
//                out.centerOfMass[climberId] = (packed_half3)in.centerOfMass[climberId];
                out.inertiaTensor[climberId] = (half3x3)in.inertiaTensor[climberId];
            }
            prev = range;
        }
//        threadgroup_barrier(mem_flags::mem_device);
    }
}
