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
    device uchar *childCount;
    device uchar *childIndex;
    device ushort *parentId;
    device uchar *climberCount;
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
      half m, packed_half3 f, packed_half3 pivot, packed_half3 t, packed_half3 c, half3x3 it,
      const UpdateCompositeBodiesIn in,
      const UpdateCompositeBodiesOut out)
{
                    ushort climberCount = in.climberCount[id];
                    if (climberCount == 0) return;
    //                int firstClimberId = in.firstClimberId[id];

                    half mass;
                    packed_half3 force, torque, centerOfMass;
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
    packed_half3 force = in.force[id];
    packed_half3 pivot = in.pivot[id];
    packed_half3 torque = in.torque[id];
    packed_half3 centerOfMass = mass * in.centerOfMass[id];

    int firstChildId = childInfo.y;
    for (ushort i = 0; i < childInfo.x; i++) {
//        int childId = firstChildId + i;
        int childId = id + 29523 + i;

        mass += in.children.mass[childId];
        force += half3(in.children.force[childId]);
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
                      device uchar *in_childCount,
                      device uchar *in_childIndex,
                      device ushort *in_parentId,
                      device uchar *in_climberCount,
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

                      constant uint * upperBound,
                      constant ushort * deltas,
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
        .childCount = in_childCount,
        .childIndex = in_childIndex,
        .parentId = in_parentId,
        .climberCount = in_climberCount,
        .mass = in_mass,
        .pivot = in_pivot,
        .force = in_force,
        .torque = in_torque,
        .centerOfMass = in_centerOfMass,
        .inertiaTensor = in_inertiaTensor
    };

    uint prev;
    uint start = *upperBound;
    ushort delta = deltas[0];
    for (ushort i = 0; i < rangeCount; i++) {
        const uint lowerBound = start - delta;
        const uint upperBound = start;
        if (gid < delta) {
            const int id = lowerBound + gid;
            const uchar childCount = in.childCount[id];
            const uchar childIndex = in.childIndex[id];
            const ushort parentId = in.parentId[id];

            half mass = in.mass[id];
            packed_half3 force = in.force[id];
            packed_half3 torque = in.torque[id];
            packed_half3 centerOfMass = in.centerOfMass[id];
            half3x3 inertiaTensor = in.inertiaTensor[id];

            for (uchar j = 0; j < childCount; j++) {
//                int childId = gid + prev.x + i * (prev.y - prev.x) / 3;
                int childId = j * (upperBound - lowerBound) + gid;
//                int childId = i + prev.x + gid * 3;
//                int childId = i + childInfo.y;

                mass += out.mass[childId];
                force += out.force[childId];
                torque += out.torque[childId];
                centerOfMass += out.centerOfMass[childId];
                inertiaTensor += out.inertiaTensor[childId];
            }

            ushort nextDelta = 0;
            if (parentId < USHRT_MAX) {
                nextDelta = deltas[i+1];
                const uint oid = childIndex * nextDelta + parentId;
                out.mass[oid] = mass;
                out.force[oid] = force;
                out.torque[oid] = torque;
                out.centerOfMass[oid] = centerOfMass;
                out.inertiaTensor[oid] = inertiaTensor;
            }

            const uchar climberCount = in.climberCount[id];
            for (uchar i = 0; i < climberCount; i++) {
                int climberId = gid + 29523 + i * (206662-29523)/6;
//                int climberId = i + 29523 + gid * 6;
//                out.mass[climberId] = in.mass[climberId];
//                out.force[climberId] = in.force[climberId];
//                out.torque[climberId] = in.torque[climberId];
//                out.centerOfMass[climberId] = in.centerOfMass[climberId];
                out.inertiaTensor[climberId] = in.inertiaTensor[climberId];
            }
            prev = lowerBound;
            start = lowerBound;
            delta = nextDelta;
        }
        threadgroup_barrier(mem_flags::mem_device);
    }
}
