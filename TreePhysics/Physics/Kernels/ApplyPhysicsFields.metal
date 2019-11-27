#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
#import "PerlinNoise.metal"

using namespace metal;

constant int fieldCount [[ function_constant(FunctionConstantIndexPhysicsFieldCount) ]];

struct ApplyPhysicsFieldsIn {
    device half *mass;
    device packed_half3 *centerOfMass;
    device packed_half3 *localPivot;
    device quath *orientation;
    device packed_half3 *velocity;
    device half *area;
    device ShapeType *shape;
};

bool appliesTo(const PhysicsField field, const float3 centerOfMass) {
    const float3 rel = metal::abs((float3)field.position - centerOfMass);
    return rel.x <= field.halfExtent.x && rel.y <= field.halfExtent.y && rel.z <= field.halfExtent.z;
}

float2x3 apply(const GravityField gravity, const float3 centerOfMass, const ApplyPhysicsFieldsIn in, const uint id, const float time) {
    float3 force = (float)in.mass[id] * (float3)gravity.g;
    return float2x3(force, float3(0));
}

float2x3 apply(const WindField wind, const float3 centerOfMass, const ApplyPhysicsFieldsIn in, const uint id, const float time) {
    PerlinNoise intensity = PerlinNoise(2, 4, 0.75, 2.5, 0);
    PerlinNoise turn = PerlinNoise(1, 2, 0.5, 2, 0);

    float3 windVelocity = (float3)wind.windVelocity;
    windVelocity = quat_act(normalize(quaternion(0, 1, 0, turn.value(float2(centerOfMass.xy + time)))), windVelocity);
    windVelocity *= intensity.value(centerOfMass.xy + time);

    const float3 relativeVelocity = windVelocity - (float3)in.velocity[id];

    float3 force, torque, normal, relativeVelocity_normal;
    quatf orientation = (quatf)in.orientation[id];
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
            float k = wind.airResistanceMultiplier * in.mass[id];
            float3 drag = k * (relativeVelocity_normal + relativeVelocity_tangential / wind.normal2tangentialDragCoefficientRatio);
            force = lift + drag;

            float l = wind.leafScale * wind.airDensity * (area / 2) * sqrt(area / M_PI_F) * dot(relativeVelocity, normal);
            float c;
            float s = sincos(wind.phi, c);
            torque = l * cross(normal, relativeVelocity * c + cross(normal, relativeVelocity * s));

            break;
    }
    return float2x3(force, torque);
}

float2x3 apply(const PhysicsField field, const float3 centerOfMass, const ApplyPhysicsFieldsIn in, const uint id, const float time) {
    float2x3 result;
    switch (field.type) {
        case PhysicsFieldTypeGravity:
            result = apply(field.gravity, centerOfMass, in, id, time);
            break;
        case PhysicsFieldTypeWind:
            result = apply(field.wind, centerOfMass, in, id, time);
            break;
    }
    return result;
}

kernel void
applyPhysicsFields(
                   constant PhysicsField *fields,

                   device half           *in_mass,
                   device packed_half3   *in_centerOfMass,
                   device packed_half3   *in_localPivot,
                   device quath          *in_orientation,
                   device packed_half3   *in_velocity,
                   device half           *in_area,
                   device ShapeType      *in_shape,

                   device packed_half3   *out_force,
                   device packed_half3   *out_torque,

                   constant float & time,
                   uint gid [[ thread_position_in_grid ]])
{
    ApplyPhysicsFieldsIn in = {
        .mass = in_mass,
        .centerOfMass = in_centerOfMass,
        .localPivot = in_localPivot,
        .orientation = in_orientation,
        .velocity = in_velocity,
        .area = in_area,
        .shape = in_shape,
    };
    float2x3 result = float2x3(0);
    for (int i = 0; i < fieldCount; i++) {
        const PhysicsField field = fields[i];
        float3 centerOfMass = (float3)in.centerOfMass[gid];
        if (appliesTo(field, centerOfMass)) {
            float2x3 forcetorque = apply(field, centerOfMass, in, gid, time);
            result += forcetorque;
            float3 torque = cross(quat_act((quatf)in.orientation[gid], (float3)-in.localPivot[gid]), forcetorque[0]);
            result[1] += torque;
        }
    }
    out_force[gid] = (half3)result[0];
    out_torque[gid] = (half3)result[1];
}
