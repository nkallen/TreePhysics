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
    device float3 *pivot;
    device float3 *centerOfMass;
    device float3x3 *inertiaTensor;
} UpdateCompositeBodiesOut;

typedef struct {
    device uchar *childCount;
    device uchar *childIndex;
    device ushort *parentId;
    device uchar *climberCount;
    device float *mass;
    device float3 *pivot;
    device float3 *force;
    device float3 *torque;
    device float3 *centerOfMass;
    device float3x3 *inertiaTensor;
    UpdateCompositeBodiesOut children;
} UpdateCompositeBodiesIn;

kernel void
updateCompositeBodies(
                      device uchar *in_childCount,
                      device uchar *in_childIndex,
                      device ushort *in_parentId,
                      device uchar *in_climberCount,
                      device float *in_mass,
                      device float3 *in_pivot,
                      device float3 *in_force,
                      device float3 *in_torque,
                      device float3 *in_centerOfMass,
                      device float3x3 *in_inertiaTensor,

                      device float *out_mass,
                      device float3 *out_force,
                      device float3 *out_torque,
                      device float3 *out_pivot,
                      device float3 *out_centerOfMass,
                      device float3x3 *out_inertiaTensor,

                      constant uint * upperBound,
                      constant uchar * maxClimberCount,
                      constant ushort * deltas,
                      uint gid [[ thread_position_in_grid ]])
{
    UpdateCompositeBodiesOut children = {
        .mass = out_mass,
        .force = out_force,
        .torque = out_torque,
        .centerOfMass = out_centerOfMass,
        .pivot = out_pivot,
        .inertiaTensor = out_inertiaTensor
    };

    UpdateCompositeBodiesIn bodies = {
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

    uint start = *upperBound;
    uint offset = *upperBound + 1;
    for (ushort i = 0; i < rangeCount; i++) {
        ushort delta = deltas[i];
        const uint lowerBound = start - delta;
        if (gid < delta) {
            const int id = lowerBound + gid;
            const uchar childCount = bodies.childCount[id];
            const uchar childIndex = bodies.childIndex[id];
            const ushort parentId = bodies.parentId[id];

            const float parentMass = bodies.mass[id];
            const float3 parentForce = bodies.force[id];
            const float3 parentTorque = bodies.torque[id];
            const float3 parentCenterOfMass = bodies.centerOfMass[id];
            const float3 parentPivot = bodies.pivot[id];

            float totalMass = parentMass;
            float3 totalForce = parentForce;
            float3 totalTorque = parentTorque;
            float3 previousCenterOfMass, totalCenterOfMass = previousCenterOfMass = parentCenterOfMass;
            float3x3 totalInertiaTensor = bodies.inertiaTensor[id];

            // Step 1: Accumulate children
            for (uchar j = 0; j < childCount; j++) {
                const int nextId = j * delta + gid;
                const float nextMass = children.mass[nextId];
                const float3 nextForce = children.force[nextId];
                const float3 nextCenterOfMass = children.centerOfMass[nextId];

                totalForce += nextForce;
                totalTorque += cross(children.pivot[nextId] - parentPivot, nextForce) +  children.torque[nextId];
                totalCenterOfMass = (totalMass * totalCenterOfMass + nextMass * nextCenterOfMass) / (totalMass + nextMass);
                totalInertiaTensor -= totalMass * sqr(skew(previousCenterOfMass - totalCenterOfMass));
                totalInertiaTensor += children.inertiaTensor[nextId] - nextMass * sqr(skew(nextCenterOfMass - totalCenterOfMass));
                totalMass += nextMass;
                previousCenterOfMass = totalCenterOfMass;
            }

            // Step 2: Walk up any "climber" chains
            float3 currentPivot = parentPivot;
            const uchar climberCount = bodies.climberCount[id];
            for (uchar j = 0; j < climberCount; j++) {
                const int nextId = offset + j * delta + gid;
                const float nextMass = bodies.mass[nextId];
                const float3 nextForce = bodies.force[nextId];
                const float3 nextCenterOfMass = bodies.centerOfMass[nextId];
                const float3 nextPivot = bodies.pivot[nextId];

                totalForce += nextForce;
                totalTorque += cross(nextPivot - currentPivot, nextForce) + bodies.torque[nextId];
                totalCenterOfMass = (totalMass * totalCenterOfMass + nextMass * nextCenterOfMass) / (totalMass + nextMass);
                totalInertiaTensor -= totalMass * sqr(skew(previousCenterOfMass - totalCenterOfMass));
                totalInertiaTensor += bodies.inertiaTensor[nextId] - nextMass * sqr(skew(nextCenterOfMass - totalCenterOfMass));
                totalMass += nextMass;
                previousCenterOfMass = totalCenterOfMass;
                currentPivot = nextPivot;
            }

            // Step 3: Store (intermediate) result for computation at the next level
            const ushort nextDelta = deltas[i+1];
            const uint oid = childIndex * nextDelta + parentId;
            children.mass[oid] = totalMass;
            children.force[oid] = totalForce;
            children.torque[oid] = totalTorque;
            children.pivot[oid] = parentPivot;
            children.centerOfMass[oid] = totalCenterOfMass;
            children.inertiaTensor[oid] = totalInertiaTensor;

            offset += delta * *maxClimberCount;
            start = lowerBound;
        }
        threadgroup_barrier(mem_flags::mem_device);
    }
}
