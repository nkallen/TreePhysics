#include <metal_stdlib>
#import "ShaderTypes.h"
#import "Math.metal"
using namespace metal;

inline half3x3 joint_worldToLocalRotation(
                                           JointStruct joint,
                                           RigidBodyStruct parentRigidBody)
{
    return transpose(parentRigidBody.rotation);
}

inline half3 joint_rotateVector(
                                 JointStruct joint,
                                 RigidBodyStruct parentRigidBody,
                                 half3 vector)
{
    return joint_worldToLocalRotation(joint, parentRigidBody) * vector;
}

inline half3x3 joint_rotateTensor(
                                   JointStruct joint,
                                   RigidBodyStruct parentRigidBody,
                                   half3x3 tensor)
{
    return joint_worldToLocalRotation(joint, parentRigidBody) * tensor * transpose(joint_worldToLocalRotation(joint, parentRigidBody));
}

inline half3 joint_position(
                             JointStruct joint,
                             RigidBodyStruct parentRigidBody)
{
    return parentRigidBody.position + parentRigidBody.rotation * half3(0, parentRigidBody.length, 0);
}

inline JointStruct
updateJoint(
            JointStruct joint,
            RigidBodyStruct parentRigidBody,
            CompositeBodyStruct childCompositeBody,
            half time)
{
    half3 pr = joint_rotateVector(joint, parentRigidBody, childCompositeBody.centerOfMass - joint_position(joint, parentRigidBody));

    half3x3 inertiaTensor_jointSpace = joint_rotateTensor(joint, parentRigidBody, childCompositeBody.inertiaTensor) -
    childCompositeBody.mass * sqr(crossMatrix(pr));
    half3 torque_jointSpace = joint_rotateVector(joint, parentRigidBody, childCompositeBody.torque);

    if (joint.k < 0) {
        // static bodies, like the root of the tree
        joint.θ = half3x3(0);
    } else {
        // Solve: Iθ'' + (αI + βK)θ' + Kθ = τ; where I = inertia tensor, τ = torque,
        // K is a spring stiffness matrix, θ = euler angles of the joint,
        // θ' = angular velocities (i.e., first derivative), etc.

        // 1. First we need to diagonalize I and K (so we can solve the diff equations) --
        // i.e., produce the generalized eigendecomposition of I and K

        // 1.a. the cholesky decomposition of I
        half3x3 L = cholesky(inertiaTensor_jointSpace);
        half3x3 L_inverse = inverse(L);
        half3x3 L_transpose_inverse = inverse(transpose(L));

        // 1.b. the generalized eigenvalue problem A * X = X * Λ
        // where A = L^(−1) * K * L^(−T); note: A is (approximately) symmetric
        half3x3 A = L_inverse * (joint.k * half3x3(1)) * L_transpose_inverse;
        half4 q = diagonalize(A);
        half3x3 Λ_M = transpose(qmat(q)) * A * qmat(q);
        half3 Λ = half3(Λ_M[0][0], Λ_M[1][1], Λ_M[2][2]);
        half3x3 X = qmat(q);

        // 2. Now we can restate the differential equation in terms of other (diagonal)
        // values: Θ'' + βΛΘ' + ΛΘ = U^T τ, where Θ = U^(-1) θ

        half3x3 U = L_transpose_inverse * X;
        half3x3 U_transpose = transpose(U);
        half3x3 U_inverse = inverse(U);

        half3 torque_diagonal = U_transpose * torque_jointSpace;
        half3 θ_diagonal_0 = U_inverse * joint.θ[0];
        half3 θ_ddt_diagonal_0 = U_inverse * joint.θ[1];
        half3 βΛ = 0.02 * Λ; // FIXME Tree.B

        // 2.a. thanks to diagonalization, we now have three independent 2nd-order
        // differential equations, θ'' + bθ' + kθ = f

        half3 solution_i = evaluateDifferential(1.0h, βΛ.x, Λ.x, torque_diagonal.x, θ_diagonal_0.x, θ_ddt_diagonal_0.x, time);
        half3 solution_ii = evaluateDifferential(1.0h, βΛ.y, Λ.y, torque_diagonal.y, θ_diagonal_0.y, θ_ddt_diagonal_0.y, time);
        half3 solution_iii = evaluateDifferential(1.0h, βΛ.z, Λ.z, torque_diagonal.z, θ_diagonal_0.z, θ_ddt_diagonal_0.z, time);

        half3x3 θ_diagonal = transpose(half3x3(solution_i, solution_ii, solution_iii));

        joint.θ = U * θ_diagonal;
    }
    return joint;
}

kernel void
updateJoints(
             device JointStruct * joints [[ buffer(BufferIndexJoints) ]],
             device RigidBodyStruct * rigidBodies [[ buffer(BufferIndexRigidBodies) ]],
             device CompositeBodyStruct * compositeBodies [[ buffer(BufferIndexCompositeBodies) ]],
             constant float * time [[ buffer(BufferIndexTime) ]],
             uint gid [[ thread_position_in_grid ]])
{
    JointStruct joint = joints[gid];
    RigidBodyStruct rigidBody = rigidBodies[gid];

    if (rigidBody.parentId != -1) {
        RigidBodyStruct parentRigidBody = rigidBodies[rigidBody.parentId];
        CompositeBodyStruct compositeBody = compositeBodies[gid];
        joint = updateJoint(joint, parentRigidBody, compositeBody, *time);
        joints[gid] = joint;
    }
}
