#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
#import "Debug.metal"
using namespace metal;

constant int rangeCount [[ function_constant(FunctionConstantIndexRangeCount) ]];

typedef struct {
    device half *mass;
    device half3 *force;
    device half3 *torque;
    device half3 *pivot;
    device half3 *centerOfMass;
    device half3x3 *inertiaTensor;
} Children;

typedef struct {
    device uchar *childCount;
    device uchar *childIndex;
    device ushort *parentId;
    device uchar *climberCount;

    device half *mass;
    device half3 *force;
    device half3 *torque;
    device half3 *centerOfMass;
    device half3 *pivot;
    device half3x3 *inertiaTensor;
    device quatf *rotation;

    device half *stiffness;
    device half *damping;
    device half3x3 *theta;
} Bodies;

inline half3x3
updateJoint(
            uint id,
            Bodies in,
            half mass,
            half3 torque,
            half3 centerOfMass,
            half3 pivot,
            half3x3 inertiaTensor,
            float time)
{
    quatf rotation = in.rotation[id];
    float3x3 rotationMatrix = float3x3_from_quat(rotation);
    quatf inverseRotation = quat_inverse(rotation);
    float3 pr = quat_act(inverseRotation, (float3)(centerOfMass - pivot));
    float3x3 inertiaTensor_jointSpace = transpose(rotationMatrix) * (float3x3)inertiaTensor * rotationMatrix;
    inertiaTensor_jointSpace -= (float)mass * sqr(skew(pr));
    float3 torque_jointSpace = quat_act(inverseRotation, (float3)torque);

    return (half3x3)inertiaTensor_jointSpace;

    // Solve: Iθ'' + (αI + βK)θ' + Kθ = τ; where I = inertia tensor, τ = torque,
    // K is a spring stiffness matrix, θ = euler angles of the joint,
    // θ' = angular velocities (i.e., first derivative), etc.

    // 1. First we need to diagonalize I and K (so we can solve the diff equations) --
    // i.e., produce the generalized eigendecomposition of I and K

    // 1.a. the cholesky decomposition of I
    float3x3 L = cholesky(inertiaTensor_jointSpace);
    float3x3 L_inverse = inverse_lowerTriangular(L);
    float3x3 L_transpose_inverse = transpose(L_inverse);

    // 1.b. the generalized eigenvalue problem A * X = X * Λ
    // where A = L^(−1) * K * L^(−T); note: A is (approximately) symmetric
    float3x3 A = L_inverse * (float3x3(in.stiffness[id])) * L_transpose_inverse;

    float4 q = diagonalize(A);
    float3x3 X = qmat(q);
    float3x3 Λ_M = transpose(X) * A * X;
    float3 Λ = float3(Λ_M[0][0], Λ_M[1][1], Λ_M[2][2]);

    // 2. Now we can restate the differential equation in terms of other (diagonal)
    // values: Θ'' + βΛΘ' + ΛΘ = U^T τ, where Θ = U^(-1) θ

    float3x3 U = L_transpose_inverse * X;
    float3x3 U_transpose = transpose(U);
    float3x3 U_inverse = inverse(U);

    float3 torque_diagonal = U_transpose * torque_jointSpace;
    float3x3 θ = float3x3(0); //in.theta[id]; FIXME
    float3 θ_diagonal_0 = U_inverse * θ[0];
    float3 θ_ddt_diagonal_0 = U_inverse * θ[1];
    float3 βΛ = in.damping[id] * Λ;

    // 2.a. thanks to diagonalization, we now have three independent 2nd-order
    // differential equations, θ'' + bθ' + kθ = f

    float3 solution_i = evaluateDifferential(1.0, βΛ.x, Λ.x, torque_diagonal.x, θ_diagonal_0.x, θ_ddt_diagonal_0.x, time);
    float3 solution_ii = evaluateDifferential(1.0, βΛ.y, Λ.y, torque_diagonal.y, θ_diagonal_0.y, θ_ddt_diagonal_0.y, time);
    float3 solution_iii = evaluateDifferential(1.0, βΛ.z, Λ.z, torque_diagonal.z, θ_diagonal_0.z, θ_ddt_diagonal_0.z, time);

    float3x3 θ_diagonal = transpose(float3x3(solution_i, solution_ii, solution_iii));

    return (half3x3)(U * θ_diagonal);
}

kernel void
updateCompositeBodies(
                      device uchar    *in_childCount,
                      device uchar    *in_childIndex,
                      device ushort   *in_parentId,
                      device uchar    *in_climberCount,
                      device half    *in_mass,
                      device half3   *in_pivot,
                      device half3   *in_force,
                      device half3   *in_torque,
                      device half3   *in_centerOfMass,
                      device half3x3 *in_inertiaTensor,

                      device half    *in_stiffness,
                      device half    *in_damping,
                      device quatf    *in_rotation,

                      device half    *out_mass,
                      device half3   *out_force,
                      device half3   *out_torque,
                      device half3   *out_pivot,
                      device half3   *out_centerOfMass,
                      device half3x3 *out_inertiaTensor,

                      device half3x3 *out_theta,

                      constant uint * upperBound,
                      constant uchar * maxClimberCount,
                      constant ushort * deltas,
                      uint gid [[ thread_position_in_grid ]])
{
    Children children = {
        .mass = out_mass,
        .force = out_force,
        .torque = out_torque,
        .centerOfMass = out_centerOfMass,
        .pivot = out_pivot,
        .inertiaTensor = out_inertiaTensor
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
        .damping = in_damping,
        .stiffness = in_stiffness,
        .theta = out_theta
    };

    uint start = *upperBound;
    uint offset = *upperBound + 1;
    for (ushort i = 0; i < rangeCount - 1; i++) {
        ushort delta = deltas[i];
        const uint lowerBound = start - delta;
        if (gid < delta) {
            const int id = lowerBound + gid;
            const uchar childCount = bodies.childCount[id];
            const uchar childIndex = bodies.childIndex[id];
            const ushort parentId = bodies.parentId[id];

            const half parentMass = bodies.mass[id];
            const half3 parentForce = bodies.force[id];
            const half3 parentTorque = bodies.torque[id];
            const half3 parentCenterOfMass = bodies.centerOfMass[id];
            const half3 parentPivot = bodies.pivot[id];

            half totalMass = parentMass;
            half3 totalForce = parentForce;
            half3 totalTorque = parentTorque;
            half3 previousCenterOfMass, totalCenterOfMass = previousCenterOfMass = parentCenterOfMass;
            half3x3 totalInertiaTensor = bodies.inertiaTensor[id];

            // Step 1: Accumulate children
            for (uchar j = 0; j < childCount; j++) {
                const int nextId = j * delta + gid;
                const half nextMass = children.mass[nextId];
                const half3 nextForce = children.force[nextId];
                const half3 nextCenterOfMass = children.centerOfMass[nextId];

                totalForce += nextForce;
                totalTorque += cross(children.pivot[nextId] - parentPivot, nextForce) +  children.torque[nextId];
                totalCenterOfMass = (totalMass * totalCenterOfMass + nextMass * nextCenterOfMass) / (totalMass + nextMass);
                totalInertiaTensor -= totalMass * sqr(skew(previousCenterOfMass - totalCenterOfMass));
                totalInertiaTensor += children.inertiaTensor[nextId] - nextMass * sqr(skew(nextCenterOfMass - totalCenterOfMass));
                totalMass += nextMass;
                previousCenterOfMass = totalCenterOfMass;
            }

            const half3x3 theta = updateJoint(id, bodies, totalMass, totalTorque, totalCenterOfMass, parentPivot, totalInertiaTensor, 1.0/60);
            out_theta[id] = theta;

            // Step 2: Walk up any "climber" chains
            half3 previousPivot = parentPivot;
            const uchar climberCount = bodies.climberCount[id];
            for (uchar j = 0; j < climberCount; j++) {
                const int nextId = offset + j * delta + gid;
                const half nextMass = bodies.mass[nextId];
                const half3 nextForce = bodies.force[nextId];
                const half3 nextCenterOfMass = bodies.centerOfMass[nextId];
                const half3 nextPivot = bodies.pivot[nextId];

                totalTorque += cross(previousPivot - nextPivot, totalForce) + bodies.torque[nextId];
                totalForce += nextForce;
                totalCenterOfMass = (totalMass * totalCenterOfMass + nextMass * nextCenterOfMass) / (totalMass + nextMass);
                totalInertiaTensor -= totalMass * sqr(skew(previousCenterOfMass - totalCenterOfMass));
                totalInertiaTensor += bodies.inertiaTensor[nextId] - nextMass * sqr(skew(nextCenterOfMass - totalCenterOfMass));
                totalMass += nextMass;

                const half3x3 theta = updateJoint(nextId, bodies, totalMass, totalTorque, totalCenterOfMass, nextPivot, totalInertiaTensor, 1.0/60);
                out_theta[nextId] = theta;

                previousCenterOfMass = totalCenterOfMass;
                previousPivot = nextPivot;
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
