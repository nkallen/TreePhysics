#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
#import "Debug.metal"

using namespace metal;

constant int rangeCount [[ function_constant(FunctionConstantIndexRangeCount) ]];

typedef struct {
    device float *mass;
    device packed_float3 *force;
    device packed_float3 *torque;
    device packed_float3 *pivot;
    device packed_float3 *centerOfMass;
    device InertiaTensor *inertiaTensor;
} Children;

typedef struct {
    device uchar *childCount;
    device uchar *childIndex;
    device uint *parentId;

    device float *mass;
    device packed_float3 *force;
    device packed_float3 *torque;
    device packed_float3 *centerOfMass;
    device packed_float3 *pivot;
    device InertiaTensor *inertiaTensor;
    device quath *jointOrientation;
    device ShapeType *shape;
} Bodies;

typedef struct {
    device uint *index;
    device float *mass;
    device packed_float3 *force;
    device packed_float3 *torque;
    device InertiaTensor *inertiaTensor;
} Free;

typedef struct {
    device InertiaTensor *inertiaTensor;
    device packed_float3 *torque;
} Out;

inline void
jointSpace(
           uint id,
           Out out,
           quatf jointOrientation,
           float mass,
           float3 torque,
           float3 centerOfMass,
           float3 pivot,
           float3x3 inertiaTensor)
{
    float3x3 rotationMatrix = float3x3_from_quat(jointOrientation);
    quatf inverseRotation = quat_inverse(jointOrientation);
    float3 pr = quat_act(inverseRotation, centerOfMass - pivot);
    float3x3 inertiaTensor_jointSpace = transpose(rotationMatrix) * inertiaTensor * rotationMatrix;
    inertiaTensor_jointSpace -= mass * sqr(skew(pr));
    float3 torque_jointSpace = quat_act(inverseRotation, torque);
    
    out.inertiaTensor[id] = inertiaTensor_from_float3x3(inertiaTensor_jointSpace);
    out.torque[id] = (packed_float3)torque_jointSpace;
}

kernel void
updateCompositeBodies(
                      device uchar          *in_childCount,
                      device uchar          *in_childIndex,
                      device uint           *in_parentId,
                      device float           *in_mass,
                      device packed_float3   *in_pivot,
                      device packed_float3   *in_force,
                      device packed_float3   *in_torque,
                      device packed_float3   *in_centerOfMass,
                      device InertiaTensor  *in_inertiaTensor,
                      device quath          *in_jointOrientation,
                      device ShapeType      *in_shape,

                      device float           *child_mass,
                      device packed_float3   *child_force,
                      device packed_float3   *child_torque,
                      device packed_float3   *child_pivot,
                      device packed_float3   *child_centerOfMass,
                      device InertiaTensor  *child_inertiaTensor,
                      
                      device packed_float3   *out_torque,
                      device InertiaTensor  *out_inertiaTensor,

                      device uint           *free_index,
                      device float           *free_mass,
                      device packed_float3   *free_force,
                      device packed_float3   *free_torque,
                      device InertiaTensor  *free_inertiaTensor,

                      device atomic_uint    &toBeFreedCount,

                      constant uint         &upperBound,
                      constant ushort       *deltas,

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
        .mass = in_mass,
        .pivot = in_pivot,
        .force = in_force,
        .torque = in_torque,
        .centerOfMass = in_centerOfMass,
        .jointOrientation = in_jointOrientation,
        .inertiaTensor = in_inertiaTensor,
        .shape = in_shape,
    };

    Free free = {
        .index = free_index,
        .mass = free_mass,
        .force = free_force,
        .torque = free_torque,
        .inertiaTensor = free_inertiaTensor,
    };
    
    Out out = {
        .inertiaTensor = out_inertiaTensor,
        .torque = out_torque,
    };
    
    uint previousLowerBound = upperBound;
    for (ushort i = 0; i < rangeCount - 1; i++) {
        ushort delta = deltas[i];
        const uint lowerBound = previousLowerBound - delta;

        if (gid < delta) {
            const int id = lowerBound + gid;
            uchar childCount = bodies.childCount[id];
            const uchar childIndex = bodies.childIndex[id];
            uint parentId  = bodies.parentId[id];
            
            const float parentMass             = (float)bodies.mass[id];
            const float3 parentForce           = (float3)bodies.force[id];
            const float3 parentTorque          = (float3)bodies.torque[id];
            const float3 parentCenterOfMass    = (float3)bodies.centerOfMass[id];
            const float3 parentPivot           = (float3)bodies.pivot[id];
            const float3x3 parentInertiaTensor = float3x3_from_inertiaTensor(bodies.inertiaTensor[id]);
            const ShapeType parentShape        = bodies.shape[id];

            float totalMass                 = parentMass;
            float3 totalForce               = parentForce;
            float3 totalTorque              = parentTorque;
            float3 totalCenterOfMass        = parentCenterOfMass;

            float3x3 totalInertiaTensor = parentInertiaTensor;

            // Step 1: Accumulate children
            for (uchar j = 0; j < childCount; j++) {
                const int childId = previousLowerBound + j * delta + gid;

                const float childMass = (float)children.mass[childId];
                const float3 childForce = (float3)children.force[childId];
                const float3 childTorque = (float3)children.torque[childId];
                const float3 childCenterOfMass = (float3)children.centerOfMass[childId];
                float3x3 childInertiaTensor = float3x3_from_inertiaTensor(children.inertiaTensor[childId]);

                const float prevTotalMass = totalMass;
                const float3 previousCenterOfMass = totalCenterOfMass;

                totalForce += childForce;
                totalTorque += cross((float3)children.pivot[childId] - parentPivot, childForce) + childTorque;
                totalMass += childMass;
                totalCenterOfMass = (prevTotalMass * previousCenterOfMass + childMass * childCenterOfMass) / totalMass;
                totalInertiaTensor -= prevTotalMass * sqr(skew(previousCenterOfMass - totalCenterOfMass));
                totalInertiaTensor += childInertiaTensor - childMass * sqr(skew(childCenterOfMass - totalCenterOfMass));
            }

            jointSpace(id, out, (quatf)bodies.jointOrientation[id], totalMass, totalTorque, totalCenterOfMass, parentPivot, totalInertiaTensor);

            // Step 2: Store (intermediate) result for computation at the next level
            const ushort nextDelta = deltas[i+1];
            if (parentId != NO_PARENT) {
                const uint oid = childIndex * nextDelta + parentId;
                children.mass[oid] = (float)totalMass;
                children.force[oid] = (float3)totalForce;
                children.torque[oid] = (float3)totalTorque;
                children.inertiaTensor[oid] = inertiaTensor_from_float3x3(totalInertiaTensor);
                children.pivot[oid] = (float3)parentPivot;
                children.centerOfMass[oid] = (float3)totalCenterOfMass;

                if (parentShape == ShapeTypeLeaf && length_squared(totalTorque) > sqr(110.5)) {
                    uint freeBodyId = atomic_fetch_add_explicit(&toBeFreedCount, 1, memory_order_relaxed);
                    free.index[freeBodyId] = id;
                }
            } else {
                free.mass[id]   = (float)totalMass;
                free.force[id]  = (float3)totalForce;
                free.torque[id] = (float3)totalTorque;
                free.inertiaTensor[id] = inertiaTensor_from_float3x3(totalInertiaTensor);
            }
        }
        previousLowerBound = lowerBound;
        threadgroup_barrier(mem_flags::mem_device);
    }
}
