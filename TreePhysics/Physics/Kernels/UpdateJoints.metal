#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"

using namespace metal;

struct UpdateJointsIn {
    device float3x3 *theta;
    device float *stiffness;
    device float *damping;
    device quatf *rotation;
    device float3 *pivot;
    device float *mass;
    device float3 *torque;
    device float3 *centerOfMass;
    device float3x3 *inertiaTensor;
};

inline float3x3
update(
            uint id,
            UpdateJointsIn in,
            float time)
{
    quatf rotation = in.rotation[id];
    float3x3 rotationMatrix = float3x3_from_quat(rotation);
    quatf inverseRotation = quat_inverse(rotation);
    float3 pr = (float3)quat_act(inverseRotation, (float3)in.centerOfMass[id] - (float3)in.pivot[id]);
    float3x3 inertiaTensor_jointSpace = transpose(rotationMatrix) * in.inertiaTensor[id] * rotationMatrix;
    inertiaTensor_jointSpace -= in.mass[id] * sqr(crossMatrix(pr));
    float3 torque_jointSpace = quat_act(inverseRotation, in.torque[id]);

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
    float3x3 A = L_inverse * (in.stiffness[id] * float3x3(1)) * L_transpose_inverse;

    float3x3 B = float3x3(
        float3(1, 20, 3),
        float3(20, 400, 5),
        float3(3, 5, 6000));

    EigenResult result = dsyevq3(B + (in.stiffness[id] * float3x3(1)));
    float3x3 X = result.Q;
    float3 Λ = result.w;

    X += A;

    // 2. Now we can restate the differential equation in terms of other (diagonal)
    // values: Θ'' + βΛΘ' + ΛΘ = U^T τ, where Θ = U^(-1) θ

    float3x3 U = L_transpose_inverse * X;
    float3x3 U_transpose = transpose(U);
    float3x3 U_inverse = inverse(U);

    float3 torque_diagonal = U_transpose * torque_jointSpace;
    float3x3 θ = in.theta[id];
    float3 θ_diagonal_0 = U_inverse * θ[0];
    float3 θ_ddt_diagonal_0 = U_inverse * θ[1];
    float3 βΛ = in.damping[id] * Λ;

    // 2.a. thanks to diagonalization, we now have three independent 2nd-order
    // differential equations, θ'' + bθ' + kθ = f

    float3 solution_i = evaluateDifferential(1.0, βΛ.x, Λ.x, torque_diagonal.x, θ_diagonal_0.x, θ_ddt_diagonal_0.x, time);
    float3 solution_ii = evaluateDifferential(1.0, βΛ.y, Λ.y, torque_diagonal.y, θ_diagonal_0.y, θ_ddt_diagonal_0.y, time);
    float3 solution_iii = evaluateDifferential(1.0, βΛ.z, Λ.z, torque_diagonal.z, θ_diagonal_0.z, θ_ddt_diagonal_0.z, time);

    float3x3 θ_diagonal = transpose(float3x3(solution_i, solution_ii, solution_iii));

    θ = U * θ_diagonal;
    return θ;
}

kernel void
updateJoints(
             device float3x3 *joint_theta,
             device float *joint_stiffness,
             device float *joint_damping,
             device quatf *joint_rotation,
             device float3 *rigidBody_pivot,

             device float *compositeBody_mass,
             device float3 *compositeBody_torque,
             device float3 *compositeBody_centerOfMass,
             device float3x3 *compositeBody_inertiaTensor,

             constant float * time,
             uint gid [[ thread_position_in_grid ]])
{
    UpdateJointsIn in = {
        .theta = joint_theta,
        .stiffness = joint_stiffness,
        .damping = joint_damping,
        .rotation = joint_rotation,
        .pivot = rigidBody_pivot,
        .mass = compositeBody_mass,
        .torque = compositeBody_torque,
        .centerOfMass = compositeBody_centerOfMass,
        .inertiaTensor = compositeBody_inertiaTensor
    };
    float3x3 theta = update(gid, in, *time);
    joint_theta[gid] = theta;
}
