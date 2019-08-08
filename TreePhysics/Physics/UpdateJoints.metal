#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
using namespace metal;

inline float3x3 joint_worldToLocalRotation(
                                           JointStruct joint,
                                           RigidBodyStruct parentRigidBody)
{
    return transpose(parentRigidBody.rotation);
}

inline float3 joint_rotateVector(
                                 JointStruct joint,
                                 RigidBodyStruct parentRigidBody,
                                 float3 vector)
{
    return joint_worldToLocalRotation(joint, parentRigidBody) * vector;
}

inline float3x3 joint_rotateTensor(
                                 JointStruct joint,
                                 RigidBodyStruct parentRigidBody,
                                 float3x3 tensor)
{
    return joint_worldToLocalRotation(joint, parentRigidBody) * tensor * transpose(joint_worldToLocalRotation(joint, parentRigidBody));
}

inline float3 joint_position(
                             JointStruct joint,
                             RigidBodyStruct parentRigidBody)
{
    return parentRigidBody.position + parentRigidBody.rotation * float3(0, parentRigidBody.length, 0);
}

inline void
updateJoint(
            JointStruct joint,
            RigidBodyStruct parentRigidBody,
            CompositeBodyStruct childCompositeBody,
            float time)
{
    float3 pr = joint_rotateVector(joint, parentRigidBody, childCompositeBody.centerOfMass - joint_position(joint, parentRigidBody));

    float3x3 inertiaTensor_jointSpace = joint_rotateTensor(joint, parentRigidBody, childCompositeBody.inertiaTensor) -
    childCompositeBody.mass * sqr(crossMatrix(pr));
    float3 torque_jointSpace = joint_rotateVector(joint, parentRigidBody, childCompositeBody.torque);

    if (joint.k < 0) {
        // static bodies, like the root of the tree
        joint.θ = float3x3(0);
    } else {
        // Solve: Iθ'' + (αI + βK)θ' + Kθ = τ; where I = inertia tensor, τ = torque,
        // K is a spring stiffness matrix, θ = euler angles of the joint,
        // θ' = angular velocities (i.e., first derivative), etc.

        // 1. First we need to diagonalize I and K (so we can solve the diff equations) --
        // i.e., produce the generalized eigendecomposition of I and K

        // 1.a. the cholesky decomposition of I
        float3x3 L = cholesky(inertiaTensor_jointSpace);
        float3x3 L_inverse = inverse(L);
        float3x3 L_transpose_inverse = inverse(transpose(L));

        // 1.b. the generalized eigenvalue problem A * X = X * Λ
        // where A = L^(−1) * K * L^(−T); note: A is (approximately) symmetric
        float3x3 A = L_inverse * (joint.k * float3x3(1)) * L_transpose_inverse;
        float4 q = diagonalize(A);
        float3x3 Λ_M = transpose(qmat(q)) * A * qmat(q);
        float3 Λ = float3(Λ_M[0][0], Λ_M[1][1], Λ_M[2][2]);
        float3x3 X = qmat(q);

        // 2. Now we can restate the differential equation in terms of other (diagonal)
        // values: Θ'' + βΛΘ' + ΛΘ = U^T τ, where Θ = U^(-1) θ

        float3x3 U = L_transpose_inverse * X;
        float3x3 U_transpose = transpose(U);
        float3x3 U_inverse = inverse(U);

        float3 torque_diagonal = U_transpose * torque_jointSpace;
        float3 θ_diagonal_0 = U_inverse * joint.θ[0];
        float3 θ_ddt_diagonal_0 = U_inverse * joint.θ[1];
        float3 βΛ = 0.02 * Λ; // FIXME Tree.B

        // 2.a. thanks to diagonalization, we now have three independent 2nd-order
        // differential equations, θ'' + bθ' + kθ = f

        float3 solution_i = evaluateDifferential(1, βΛ.x, Λ.x, torque_diagonal.x, θ_diagonal_0.x, θ_ddt_diagonal_0.x, time);
        float3 solution_ii = evaluateDifferential(1, βΛ.y, Λ.y, torque_diagonal.y, θ_diagonal_0.y, θ_ddt_diagonal_0.y, time);
        float3 solution_iii = evaluateDifferential(1, βΛ.z, Λ.z, torque_diagonal.z, θ_diagonal_0.z, θ_ddt_diagonal_0.z, time);

        float3x3 θ_diagonal = transpose(float3x3(solution_i, solution_ii, solution_iii));

        joint.θ = U * θ_diagonal;
    }
}

kernel void
updateJoints(
                      device CompositeBodyStruct * compositeBodies [[ buffer(BufferIndexCompositeBodies) ]],
                      uint gid [[ thread_position_in_grid ]])
{
}
