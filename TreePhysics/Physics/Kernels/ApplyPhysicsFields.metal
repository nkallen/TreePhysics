#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
#import "PerlinNoise.metal"

using namespace metal;

constant int fieldCount [[ function_constant(FunctionConstantIndexPhysicsFieldCount) ]];

struct ApplyPhysicsFieldsIn {
    device uint *parentId;
    device half *mass;
    device packed_half3 *centerOfMass;
    device InertiaTensor *inertiaTensor;
    device packed_half3 *localPivot;
    device quath *orientation;
    device packed_half3 *velocity;
    device packed_half3 *angularVelocity;
    device half *area;
    device ShapeType *shape;
};

inline bool
appliesTo(const PhysicsField field, const float3 centerOfMass) {
    const float3 rel = metal::abs((float3)field.position - centerOfMass);
    return rel.x <= field.halfExtent.x && rel.y <= field.halfExtent.y && rel.z <= field.halfExtent.z;
}

inline float2x3
apply(const GravityField gravity, const float3 centerOfMass, const ApplyPhysicsFieldsIn in, const uint id, const float time) {
    // FIXME stop loading attributes in a loop (potentially); load once at the beginning.
    float3 force = (float)in.mass[id] * (float3)gravity.g;
    return float2x3(force, float3(0));
}

inline float2x3
apply(const WindField wind, const float3 centerOfMass, const quatf orientation, const ApplyPhysicsFieldsIn in, const uint id, const float time) {
    PerlinNoise intensity = PerlinNoise(0.1, 4, 0.75, 2.5, 0);
    PerlinNoise turn = PerlinNoise(1, 2, 0.5, 2, 0);

    float3 windVelocity = (float3)wind.windVelocity;
    windVelocity = quat_act(quat_from_axis_angle(float3(0, 1, 0), turn.value(float2(centerOfMass.xy + time))), windVelocity);
    windVelocity *= intensity.value(centerOfMass.xy + time);

    const float3 relativeVelocity = windVelocity - (float3)in.velocity[id];

    float3 force, torque, normal, relativeVelocity_normal;
    float area = in.area[id];
    switch (in.shape[id]) {
        case ShapeTypeInternode:
            normal = quat_heading(orientation);
            relativeVelocity_normal = relativeVelocity - dot(relativeVelocity, normal) * normal;
            force = wind.branchScale * wind.airDensity * area * length(relativeVelocity_normal) * relativeVelocity_normal;
            torque = 0;
            break;
        case ShapeTypeLeaf:
            normal = quat_up(orientation);
            relativeVelocity_normal = dot(relativeVelocity, normal) * normal;
            float3 relativeVelocity_tangential = relativeVelocity - relativeVelocity_normal;
            float3 lift = wind.leafScale * wind.airDensity * area * length(relativeVelocity) * relativeVelocity_normal;
            float3 drag = (float)wind.airResistanceMultiplier * (float)in.mass[id] * (relativeVelocity_normal + relativeVelocity_tangential / wind.normal2tangentialDragCoefficientRatio);
            force = lift + drag;

            float l = wind.leafScale * wind.airDensity * (area / 2) * sqrt(area) / sqrt(M_PI_F) * dot(relativeVelocity, normal);
            float c, s = sincos(wind.phi, c);
            torque = l * cross(normal, relativeVelocity * c + cross(normal, relativeVelocity * s));
            torque -= (float3)wind.airResistanceMultiplier * float3x3_from_inertiaTensor(in.inertiaTensor[id]) * (float3)in.angularVelocity[id];

            break;
    }
    return float2x3(force, torque);
}

inline float2x3
apply(const PhysicsField field, const float3 centerOfMass, const quatf orientation, const ApplyPhysicsFieldsIn in, const uint id, const float time) {
    float2x3 result;
    switch (field.type) {
        case PhysicsFieldTypeGravity:
            result = apply(field.gravity, centerOfMass, in, id, time);
            break;
        case PhysicsFieldTypeWind:
            result = apply(field.wind, centerOfMass, orientation, in, id, time);
            break;
    }
    return result;
}

kernel void
applyPhysicsFields(
                   constant PhysicsField *fields,

                   device uint           *in_parentId,
                   device half           *in_mass,
                   device packed_half3   *in_centerOfMass,
                   device InertiaTensor  *in_inertiaTensor,
                   device packed_half3   *in_localPivot,
                   device quath          *in_orientation,
                   device packed_half3   *in_velocity,
                   device packed_half3   *in_angularVelocity,
                   device half           *in_area,
                   device ShapeType      *in_shape,

                   device packed_half3   *out_force,
                   device packed_half3   *out_torque,

                   constant float & time,
                   const uint gid [[ thread_position_in_grid ]])
{
    const ApplyPhysicsFieldsIn in = {
        .parentId = in_parentId,
        .mass = in_mass,
        .centerOfMass = in_centerOfMass,
        .inertiaTensor = in_inertiaTensor,
        .localPivot = in_localPivot,
        .orientation = in_orientation,
        .velocity = in_velocity,
        .angularVelocity = in_angularVelocity,
        .area = in_area,
        .shape = in_shape,
    };
    const uint parentId  = in.parentId[gid];
    const float3 centerOfMass = (float3)in.centerOfMass[gid];
    const quatf orientation = (quatf)in.orientation[gid];
    const float3 localPivot = (float3)in.localPivot[gid];

    float2x3 result = float2x3(0);
    for (int i = 0; i < fieldCount; i++) {
        const PhysicsField field = fields[i];
        if (appliesTo(field, centerOfMass)) {
            float2x3 forcetorque = apply(field, centerOfMass, orientation, in, gid, time);
            result += forcetorque;
            if (parentId != NO_PARENT) {
                float3 torque = cross(quat_act(orientation, -localPivot), forcetorque[0]);
                result[1] += torque;
            }
        }
    }
    out_force[gid] = (packed_half3)result[0];
    out_torque[gid] = (packed_half3)result[1];
}
