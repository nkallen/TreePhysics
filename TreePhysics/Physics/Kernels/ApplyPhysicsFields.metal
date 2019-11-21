#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
#import "PerlinNoise.metal"

using namespace metal;

constant int fieldCount [[ function_constant(FunctionConstantIndexPhysicsFieldCount) ]];

struct ApplyPhysicsFieldsIn {
    device half *mass;
    device packed_half3 *centerOfMass;
    device quath *orientation;
    device packed_half3 *velocity;
    device half *area;
};

bool appliesTo(const PhysicsField field, const float3 centerOfMass) {
    const float3 rel = metal::abs((float3)field.position - centerOfMass);
    return rel.x <= field.halfExtent.x && rel.y <= field.halfExtent.y && rel.z <= field.halfExtent.z;
}

half2x3 apply(const GravityField gravity, const float3 centerOfMass, const ApplyPhysicsFieldsIn in, const uint id, const float time) {
    half3 force = in.mass[id] * gravity.g;
    return half2x3(force, half3(0));
}

half2x3 apply(const WindField wind, const float3 centerOfMass, const ApplyPhysicsFieldsIn in, const uint id, const float time) {
    PerlinNoise intensity = PerlinNoise(1, 4, 0.75, 200, 0);
    PerlinNoise turn = PerlinNoise(1, 2, 0.75, 0.75, 0);

    float3 windVelocity = (float3)wind.windVelocity;
    windVelocity = quat_act(quaternion(0, 1, 0, turn.value(float2(centerOfMass.xy + time))), windVelocity);
    windVelocity *= 0.3*intensity.value(time);

    const float3 relativeVelocity = windVelocity - (float3)in.velocity[id];
    const float3 normal = quat_heading((quatf)in.orientation[id]);
    const float3 relativeVelocity_normal = relativeVelocity - dot(relativeVelocity, normal) * normal;
    const float3 force = wind.branchScale * wind.airDensity * in.area[id] * length(relativeVelocity_normal) * relativeVelocity_normal;
    return half2x3((half3)force, half3(0));
}

half2x3 apply(const PhysicsField field, const float3 centerOfMass, const ApplyPhysicsFieldsIn in, const uint id, const float time) {
    half2x3 result;
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
                   device quath          *in_orientation,
                   device packed_half3   *in_velocity,
                   device half           *in_area,

                   device packed_half3   *out_force,
                   device packed_half3   *out_torque,

                   constant float & time,
                   uint gid [[ thread_position_in_grid ]])
{
    ApplyPhysicsFieldsIn in = {
        .mass = in_mass,
        .centerOfMass = in_centerOfMass,
        .orientation = in_orientation,
        .velocity = in_velocity,
        .area = in_area,
    };
    half2x3 result = half2x3(0);
    for (int i = 0; i < fieldCount; i++) {
        const PhysicsField field = fields[i];
        float3 centerOfMass = (float3)in.centerOfMass[gid];
        if (appliesTo(field, centerOfMass)) {
            result += apply(field, centerOfMass, in, gid, time);
        }
    }
    out_force[gid] = result[0];
    out_torque[gid] = result[1];
}
