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
    device packed_half3 *pivot;
    device packed_half3 *centerOfMass;
    device InertiaTensor *inertiaTensor;
} Children;

typedef struct {
    device uchar *childCount;
    device uchar *childIndex;
    device ushort *parentId;
    device uchar *climberCount;

    device half *mass;
    device packed_half3 *force;
    device packed_half3 *torque;
    device packed_half3 *centerOfMass;
    device packed_half3 *pivot;
    device InertiaTensor *inertiaTensor;
    device quath *rotation;
} Bodies;

typedef struct {
    device InertiaTensor *inertiaTensor;
    device packed_half3 *torque;
} Out;

inline void
jointSpace(
           uint id,
           Out out,
           quatf rotation,
           float mass,
           float3 torque,
           float3 centerOfMass,
           float3 pivot,
           float3x3 inertiaTensor)
{
    float3x3 rotationMatrix = float3x3_from_quat(rotation);
    quatf inverseRotation = quat_inverse(rotation);
    float3 pr = quat_act(inverseRotation, centerOfMass - pivot);
    float3x3 inertiaTensor_jointSpace = transpose(rotationMatrix) * inertiaTensor * rotationMatrix;
    inertiaTensor_jointSpace -= mass * sqr(skew(pr));
    float3 torque_jointSpace = quat_act(inverseRotation, torque);
    
    out.inertiaTensor[id] = inertiaTensor_from_float3x3(inertiaTensor_jointSpace);
    out.torque[id] = (packed_half3)torque_jointSpace;
}

kernel void
updateCompositeBodies(
                      device uchar    *in_childCount,
                      device uchar    *in_childIndex,
                      device ushort   *in_parentId,
                      device uchar    *in_climberCount,
                      device half    *in_mass,
                      device packed_half3   *in_pivot,
                      device packed_half3   *in_force,
                      device packed_half3   *in_torque,
                      device packed_half3   *in_centerOfMass,
                      device InertiaTensor *in_inertiaTensor,
                      
                      device quath    *in_rotation,
                      
                      device half    *child_mass,
                      device packed_half3   *child_force,
                      device packed_half3   *child_torque,
                      device packed_half3   *child_pivot,
                      device packed_half3   *child_centerOfMass,
                      device InertiaTensor *child_inertiaTensor,
                      
                      device packed_half3 *out_torque,
                      device InertiaTensor *out_inertiaTensor,

                      constant uint * upperBound,
                      constant ushort * deltas,
                      uint gid [[ thread_position_in_grid ]])
{
    Children children = {
        .mass = child_mass,
        .force = child_force,
        .torque = child_torque,
        .centerOfMass = child_centerOfMass,
        .pivot = child_pivot,
        .inertiaTensor = child_inertiaTensor
    };
    
    Bodies bodies = {
        .childCount = in_childCount,
        .childIndex = in_childIndex,
        .parentId = in_parentId,
        .climberCount = in_climberCount,
        .mass = in_mass,
        .pivot = in_pivot,
        .force = in_force,
        .torque = in_torque,
        .centerOfMass = in_centerOfMass,
        .rotation = in_rotation,
        .inertiaTensor = in_inertiaTensor,
    };
    
    Out out = {
        .inertiaTensor = out_inertiaTensor,
        .torque = out_torque,
    };
    
    uint start = *upperBound;
    for (ushort i = 0; i < rangeCount - 1; i++) {
        ushort delta = deltas[i];
        const uint lowerBound = start - delta;

        if (gid < delta) {
            const int id = lowerBound + gid;
            const uchar childCount = bodies.childCount[id];
            const uchar childIndex = bodies.childIndex[id];
            const ushort parentId = bodies.parentId[id];
            
            const float parentMass = (float)bodies.mass[id];
            const float3 parentForce = (float3)bodies.force[id];
            const float3 parentTorque = (float3)bodies.torque[id];
            const float3 parentCenterOfMass = (float3)bodies.centerOfMass[id];
            const float3 parentPivot = (float3)bodies.pivot[id];
            
            float totalMass = parentMass;
            float3 totalForce = parentForce;
            float3 totalTorque = parentTorque;
            float3 previousCenterOfMass, totalCenterOfMass = previousCenterOfMass = parentCenterOfMass;
            float3x3 totalInertiaTensor = float3x3_from_inertiaTensor(bodies.inertiaTensor[id]);
            
            // Step 1: Accumulate children
            for (uchar j = 0; j < childCount; j++) {
                const int nextId = j * delta + gid;
                const float nextMass = (float)children.mass[nextId];
                const float3 nextForce = (float3)children.force[nextId];
                const float3 nextCenterOfMass = (float3)children.centerOfMass[nextId];
                
                totalForce += nextForce;
                totalTorque += cross((float3)children.pivot[nextId] - parentPivot, nextForce) + (float3)children.torque[nextId];
                totalCenterOfMass = (totalMass * totalCenterOfMass + nextMass * nextCenterOfMass) / (totalMass + nextMass);
                totalInertiaTensor -= totalMass * sqr(skew(previousCenterOfMass - totalCenterOfMass));
                totalInertiaTensor += float3x3_from_inertiaTensor(children.inertiaTensor[nextId]) - nextMass * sqr(skew(nextCenterOfMass - totalCenterOfMass));
                totalMass += nextMass;
                previousCenterOfMass = totalCenterOfMass;
            }
            
            jointSpace(id, out, (quatf)bodies.rotation[id], totalMass, totalTorque, totalCenterOfMass, parentPivot, totalInertiaTensor);

            // Step 2: Store (intermediate) result for computation at the next level
            const ushort nextDelta = deltas[i+1];
            const uint oid = childIndex * nextDelta + parentId;
            children.mass[oid] = (half)totalMass;
            children.force[oid] = (half3)totalForce;
            children.torque[oid] = (half3)totalTorque;
            children.pivot[oid] = (half3)parentPivot;
            children.centerOfMass[oid] = (half3)totalCenterOfMass;
            children.inertiaTensor[oid] = inertiaTensor_from_float3x3(totalInertiaTensor);
            
            start = lowerBound;
        }
        threadgroup_barrier(mem_flags::mem_device);
    }
}
