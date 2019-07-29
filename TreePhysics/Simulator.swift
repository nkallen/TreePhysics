import Foundation
import simd

final class Simulator {
    let tree: Tree
    let rigidBodiesLevelOrder: [RigidBody]
    let rigidBodiesReverseLevelOrder: [RigidBody]

    init(tree: Tree) {
        self.tree = tree
        self.rigidBodiesLevelOrder = tree.root.flatten.filter { $0.kind == .dynamic }
        self.rigidBodiesReverseLevelOrder = self.rigidBodiesLevelOrder.reversed()
        updateRigidBodies()
    }

    func update(at time: TimeInterval) {
        updateCompositeBodies()
        updateSprings(at: time)
        updateRigidBodies()
    }

    func updateCompositeBodies() {
        for rigidBody in rigidBodiesReverseLevelOrder {
            rigidBody.apply(force: Tree.gravity, at: 0.5) // FIXME

            let composite = rigidBody.composite

            composite.mass = rigidBody.mass
            composite.force = rigidBody.force
            composite.torque = rigidBody.torque
            composite.centerOfMass = rigidBody.mass * rigidBody.centerOfMass

            for childJoint in rigidBody.childJoints {
                let childRigidBody = childJoint.childRigidBody
                let childComposite = childRigidBody.composite

                composite.mass += childComposite.mass
                composite.force += childComposite.force

                // this is due to distributivity of cross product
                composite.torque +=
                    cross(childJoint.position - rigidBody.position, childComposite.force) + childComposite.torque

                composite.centerOfMass += childComposite.mass * childComposite.centerOfMass
            }
            composite.centerOfMass /= composite.mass

            composite.inertiaTensor = rigidBody.inertiaTensor -
                rigidBody.mass * sqr((rigidBody.centerOfMass - rigidBody.composite.centerOfMass).cross_matrix)


            for childJoint in rigidBody.childJoints {
                let childRigidBody = childJoint.childRigidBody
                let childComposite = childRigidBody.composite

                // using the parallel axis theorem I' = I + md^2, but with tensors:
                composite.inertiaTensor += childComposite.inertiaTensor -
                    childComposite.mass * sqr((childComposite.centerOfMass - composite.centerOfMass).cross_matrix)
            }
        }
    }

    func updateSprings(at time: TimeInterval) {
        for rigidBody in rigidBodiesLevelOrder { // Order does not matter
            if let parentJoint = rigidBody.parentJoint {
                let pr = parentJoint.rotate(vector: rigidBody.composite.centerOfMass - parentJoint.position)

                let inertiaTensor_jointSpace = parentJoint.rotate(tensor: rigidBody.composite.inertiaTensor) -
                    rigidBody.composite.mass * sqr(pr.cross_matrix)

//                print("t", rigidBody.composite.torque)
                let torque_jointSpace = parentJoint.rotate(vector: rigidBody.composite.torque)
//                print("tjs", torque_jointSpace)
                if parentJoint.k == .infinity {
                    parentJoint.angle = 0
                    parentJoint.angularVelocity = 0
                    parentJoint.angularAcceleration = 0
                } else {
                    // Solve: Iθ'' + (αI + βK)θ' + Kθ = τ
                    // θ(0) = joint's angle, θ'(0) = joint's angular velocity

                    // 1. First we need to diagonalize I and K so we can solve diff equation, i.e.,
                    // produce the generalized eigendecomposition of I and K

                    // 1.a. the cholesky decomposition of I
                    let L = inertiaTensor_jointSpace.cholesky
                    let L_inverse = L.inverse, L_transpose_inverse = L.transpose.inverse

                    // 1.b. the generalized eigenvalue problem A * X = X * Λ
                    // where A = L^(−1) * K * L^(−T); note: A is (approximately) symmetric
                    let A = L_inverse * (parentJoint.k * matrix_identity_float3x3) * L_transpose_inverse
                    let (Λ, X) = A.eigen_ql! // FIXME investigate analytic option, what percentage of the time it would succeed

                    // 2. Now we can restate the differential equation in terms of other (diagonal)
                    // values: Θ'' + βΛΘ' + ΛΘ = U^T τ, where Θ = U^(-1) θ

                    let U = L_transpose_inverse * X
                    let U_transpose =  U.transpose, U_inverse = U.inverse

                    let torque_diagonal = U_transpose * torque_jointSpace
                    let theta_diagonal_0 = U_inverse * float3(0,0,parentJoint.angle)
                    let theta_ddt_diagonal_0 = U_inverse * float3(0,0,0) // parentJoint.angularVelocity
                    let βΛ = Tree.B * Λ

                    // 2.a. thanks to diagonalization, we now have three independent 2nd-order
                    // differential equations, θ'' + bθ' + kθ = f 

                    // 2.a.i,
                    let solution_i = solve_differential(a: 1, b: βΛ.x, c: Λ.x, g: torque_diagonal.x, y_0: theta_diagonal_0.x, y_ddt_0: theta_ddt_diagonal_0.x)

                    let solution_ii = solve_differential(a: 1, b: βΛ.y, c: Λ.y, g: torque_diagonal.y, y_0: theta_diagonal_0.y, y_ddt_0: theta_ddt_diagonal_0.y)

                    let solution_iii = solve_differential(a: 1, b: βΛ.z, c: Λ.z, g: torque_diagonal.z, y_0: theta_diagonal_0.z, y_ddt_0: theta_ddt_diagonal_0.z)

                    let thetas_diagonal = float3(
                        evaluate(differential: solution_i,   at: Float(time)).x,
                        evaluate(differential: solution_ii,  at: Float(time)).x,
                        evaluate(differential: solution_iii, at: Float(time)).x)

                    var thetas = U * thetas_diagonal

                    parentJoint.angle = thetas.z // max(Tree.minAngle, min(Tree.maxAngle, thetas.z))
                    parentJoint.angularVelocity = thetas.y
                    parentJoint.angularAcceleration = thetas.z
                }
            }
        }
    }

    func updateRigidBodies() {
        for rigidBody in rigidBodiesLevelOrder {
            rigidBody.updateTransform()
            for joint in rigidBody.childJoints {
                joint.updateTransform()
            }
            rigidBody.resetForces()
        }
    }
}

extension RigidBody: CustomDebugStringConvertible {
    var debugDescription: String {
        return name
    }
}

extension RigidBody: Equatable, Hashable {
    static func == (lhs: RigidBody, rhs: RigidBody) -> Bool {
        return lhs === rhs
    }

    func hash(into hasher: inout Hasher) {
        hasher.combine(ObjectIdentifier(self))
    }
}
