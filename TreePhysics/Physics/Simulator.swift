import Foundation
import simd

final class Simulator {
    let tree: Tree
    let rigidBodiesLevelOrder: [RigidBody]
    let rigidBodiesReverseLevelOrder: [RigidBody]
    private var fields: [PhysicsField] = []

    init(tree: Tree) {
        self.tree = tree
        self.rigidBodiesLevelOrder = tree.root.flattened().filter { $0.kind == .dynamic }
        self.rigidBodiesReverseLevelOrder = self.rigidBodiesLevelOrder.reversed()
        updateRigidBodies()
    }

    func add(field: PhysicsField) {
        fields.append(field)
    }

    func update(at time: TimeInterval) {
        updateFields(at: time)
        updateCompositeBodies()
        updateJoints(at: time)
        updateRigidBodies()
        resetForces()
    }

    func updateFields(at time: TimeInterval) {
        for rigidBody in rigidBodiesReverseLevelOrder { // Order is unimportant
            for field in fields {
                let position = rigidBody.position
                if field.applies(to: position) {
                    let force = field.eval(position: position, velocity: float3.zero, mass: rigidBody.mass, time: time)
                    rigidBody.apply(force: force, at: 0.5)
                }
            }
        }
    }

    func updateCompositeBodies() {
        for rigidBody in rigidBodiesReverseLevelOrder {
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
                rigidBody.mass * sqr((rigidBody.centerOfMass - composite.centerOfMass).crossMatrix)

            for childJoint in rigidBody.childJoints {
                let childRigidBody = childJoint.childRigidBody
                let childComposite = childRigidBody.composite

                // using the parallel axis theorem I' = I + md^2, but with tensors:
                composite.inertiaTensor += childComposite.inertiaTensor -
                    childComposite.mass * sqr((childComposite.centerOfMass - composite.centerOfMass).crossMatrix)
            }
        }
    }

    func updateJoints(at time: TimeInterval) {
        for rigidBody in rigidBodiesReverseLevelOrder { // Order does not matter
            if let parentJoint = rigidBody.parentJoint {
                let pr = parentJoint.rotate(vector: rigidBody.composite.centerOfMass - parentJoint.position)

                let inertiaTensor_jointSpace = parentJoint.rotate(tensor: rigidBody.composite.inertiaTensor) -
                    rigidBody.composite.mass * sqr(pr.crossMatrix)
                let torque_jointSpace = parentJoint.rotate(vector: rigidBody.composite.torque)

                if parentJoint.k == .infinity {
                    // static bodies, like the root of the tree
                    parentJoint.θ = matrix_float3x3(0)
                } else {
                    // Solve: Iθ'' + (αI + βK)θ' + Kθ = τ; where I = inertia tensor, τ = torque,
                    // K is a spring stiffness matrix, θ = euler angles of the joint,
                    // θ' = angular velocities (i.e., first derivative), etc.

                    // 1. First we need to diagonalize I and K (so we can solve the diff equations) --
                    // i.e., produce the generalized eigendecomposition of I and K

                    // 1.a. the cholesky decomposition of I
                    let L = inertiaTensor_jointSpace.cholesky
                    let L_inverse = L.inverse, L_transpose_inverse = L.transpose.inverse

                    // 1.b. the generalized eigenvalue problem A * X = X * Λ
                    // where A = L^(−1) * K * L^(−T); note: A is (approximately) symmetric
                    let A = L_inverse * (parentJoint.k * matrix_identity_float3x3) * L_transpose_inverse
                    let (Λ, X) = A.eigen_ql!

                    // 2. Now we can restate the differential equation in terms of other (diagonal)
                    // values: Θ'' + βΛΘ' + ΛΘ = U^T τ, where Θ = U^(-1) θ

                    let U = L_transpose_inverse * X
                    let U_transpose = U.transpose, U_inverse = U.inverse

                    let torque_diagonal = U_transpose * torque_jointSpace
                    let θ_diagonal_0 = U_inverse * parentJoint.θ[0]
                    let θ_ddt_diagonal_0 = U_inverse * parentJoint.θ[1]
                    let βΛ = Tree.B * Λ

                    // 2.a. thanks to diagonalization, we now have three independent 2nd-order
                    // differential equations, θ'' + bθ' + kθ = f 

                    // FIXME cleanup and make similar to metal impl
                    let solution_i = solve_differential(a: 1, b: βΛ.x, c: Λ.x, g: torque_diagonal.x, y_0: θ_diagonal_0.x, y_ddt_0: θ_ddt_diagonal_0.x)
                    let solution_ii = solve_differential(a: 1, b: βΛ.y, c: Λ.y, g: torque_diagonal.y, y_0: θ_diagonal_0.y, y_ddt_0: θ_ddt_diagonal_0.y)
                    let solution_iii = solve_differential(a: 1, b: βΛ.z, c: Λ.z, g: torque_diagonal.z, y_0: θ_diagonal_0.z, y_ddt_0: θ_ddt_diagonal_0.z)

                    let θ_diagonal = matrix_float3x3(rows: [
                        evaluate(differential: solution_i, at: Float(time)),
                        evaluate(differential: solution_ii, at: Float(time)),
                        evaluate(differential: solution_iii, at: Float(time))])

                    parentJoint.θ = U * θ_diagonal
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
        }
    }

    func resetForces() {
        for rigidBody in rigidBodiesLevelOrder {
            rigidBody.resetForces()
        }
    }
}
