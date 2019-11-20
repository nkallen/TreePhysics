#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"

using namespace metal;

constant int fieldCount [[ function_constant(FunctionConstantIndexPhysicsFieldCount) ]];

bool PhysicsField::appliesTo(half3 centerOfMass) {
    half3 rel = metal::abs(position - centerOfMass);
    return rel.x <= halfExtent.x && rel.y <= halfExtent.y && rel.z <= halfExtent.z;
}

half2x3 PhysicsField::apply(half mass) {
    half3 force, torque;
    force = torque = half3(0);
    switch (type) {
        case PhysicsFieldTypeAttractor:
            break;
        case PhysicsFieldTypeGravity:
            force = mass * gravity.g;
            break;
        case PhysicsFieldTypeWind:
            break;
    }
    return half2x3(force, torque);
}

kernel void
applyPhysicsFields(
                   constant PhysicsField *fields,
                   device half           *in_mass,
                   device packed_half3   *in_centerOfMass,
                   device packed_half3   *out_force,
                   device packed_half3   *out_torque,
                   constant float * time,
                   uint gid [[ thread_position_in_grid ]])
{
    half2x3 result = half2x3(0);
    for (int i = 0; i < fieldCount; i++) {
        PhysicsField field = fields[i];
        if (field.appliesTo(in_centerOfMass[gid])) {
            result += field.apply(in_mass[gid]);
        }
    }
    out_force[gid] = result[0];
    out_torque[gid] = result[1];
}
