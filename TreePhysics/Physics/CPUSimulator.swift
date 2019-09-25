import Foundation
import simd

let torqueFictitiousMultiplier_i: Float = 0.0
let torqueFictitiousMultiplier_ii: Float = 0.0
let torqueFictitiousMultiplier_iii: Float = 0.0

// FIXME extract a World object that has add/remove methods and so forth.
public final class CPUSimulator {
    var rigidBodiesLevelOrder: [RigidBody] = []
    private var fields: [PhysicsField] = []

    public init() {}

    var rigidBodiesUnordered: [RigidBody] {
        return rigidBodiesLevelOrder
    }

    func add(rigidBody: RigidBody) {
        self.rigidBodiesLevelOrder.append(contentsOf: rigidBody.flattened())
        updateArticulatedBodies()
    }

    public func add(field: PhysicsField) {
        fields.append(field)
    }

    public func update(at time: TimeInterval) {
        updateFields(at: time)
        updateCompositeBodies()
        deArticulateBodies()
        updateJoints(at: time)
        updateArticulatedBodies()
        updateFreeBodies(at: time)
        resetForces()
    }

    let start = Date()

    func updateFields(at time: TimeInterval) {
        for rigidBody in rigidBodiesUnordered {
            for field in fields {
                if field.applies(to: rigidBody.centerOfMass) {
                    let time = Date().timeIntervalSince(start)
                    let force = field.force(rigidBody: rigidBody, time: time)
                    let torque = field.torque(rigidBody: rigidBody, time: time)
                    rigidBody.apply(force: force, torque: torque)
                }
            }
        }
    }

    func updateCompositeBodies() {
        for rigidBody in rigidBodiesLevelOrder.reversed() {
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
                    cross(childJoint.position - rigidBody.translation, childComposite.force) + childComposite.torque

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

    func deArticulateBodies() {
        for rigidBody in rigidBodiesUnordered {
            if let parentJoint = rigidBody.parentJoint {
                if length(rigidBody.torque) > parentJoint.torqueThreshold {
                    rigidBody.removeFromParent()
                }
            }
        }
    }

    func updateJoints(at time: TimeInterval) {
        let time = Float(time)
        for rigidBody in rigidBodiesUnordered {
            if let parentJoint = rigidBody.parentJoint {
                let pr = parentJoint.rotate(vector: rigidBody.composite.centerOfMass - parentJoint.position)

                let inertiaTensor_jointSpace = parentJoint.rotate(tensor: rigidBody.composite.inertiaTensor) -
                    rigidBody.composite.mass * sqr(pr.crossMatrix)

                let torque_jointSpace = parentJoint.rotate(vector: rigidBody.composite.torque)

                // Calculate any fictitious forces:
                let parentAngularAcceleration_jointSpace = parentJoint.rotate(vector: parentJoint.parentRigidBody.angularAcceleration)
                let parentAngularVelocity_jointSpace = parentJoint.rotate(vector: parentJoint.parentRigidBody.angularVelocity)
                let childAngularVelocity_jointSpace = parentJoint.rotate(vector: rigidBody.angularVelocity)

                let torqueFictitious_jointSpace_i: float3 = torqueFictitiousMultiplier_i *
                    -rigidBody.composite.mass * pr.crossMatrix * parentJoint.rotate(vector: parentJoint.acceleration)
                let torqueFictitious_jointSpace_ii: float3 = torqueFictitiousMultiplier_ii *
                    -inertiaTensor_jointSpace * (parentAngularAcceleration_jointSpace + parentAngularVelocity_jointSpace.crossMatrix * childAngularVelocity_jointSpace)
                let torqueFictitious_jointSpace_iii: float3 = torqueFictitiousMultiplier_ii *
                    -childAngularVelocity_jointSpace.crossMatrix * inertiaTensor_jointSpace * childAngularVelocity_jointSpace

                let torqueFictitious_jointSpace = torqueFictitious_jointSpace_i + torqueFictitious_jointSpace_ii + torqueFictitious_jointSpace_iii

                let torqueTotal_jointSpace = torque_jointSpace + torqueFictitious_jointSpace

                // Solve: Iθ'' + (αI + βK)θ' + Kθ = τ; where I = inertia tensor, τ = torque,
                // K is a spring stiffness matrix, θ = euler angles of the joint,
                // θ' = angular velocities (i.e., first derivative), etc.

                // 1. First we need to diagonalize I and K (so we can solve the diff equations) --
                // i.e., produce the generalized eigendecomposition of I and K

                // 1.a. the cholesky decomposition of I
                let L = inertiaTensor_jointSpace.cholesky
                let L_inverse = L.inverse, L_transpose_inverse = L_inverse.transpose

                // 1.b. the generalized eigenvalue problem A * X = X * Λ
                // where A = L^(−1) * K * L^(−T); note: A is (approximately) symmetric
                let A = L_inverse * (parentJoint.k * matrix_identity_float3x3) * L_transpose_inverse
                let (Λ, X) = A.eigen_analytical!

                // 2. Now we can restate the differential equation in terms of other (diagonal)
                // values: Θ'' + βΛΘ' + ΛΘ = U^T τ, where Θ = U^(-1) θ
                let U = L_transpose_inverse * X
                let U_transpose = U.transpose, U_inverse = U.inverse

                let torque_diagonal = U_transpose * torqueTotal_jointSpace
                let θ_diagonal_0 = U_inverse * parentJoint.θ[0]
                let θ_ddt_diagonal_0 = U_inverse * parentJoint.θ[1]
                let βΛ = Internode.β * Λ

                // 2.a. thanks to diagonalization, we now have three independent 2nd-order
                // differential equations, θ'' + bθ' + kθ = f
                let θ_diagonal = float3x3(rows: [
                    evaluateDifferential(a: 1, b: βΛ.x, c: Λ.x, g: torque_diagonal.x, y_0: θ_diagonal_0.x, y_ddt_0: θ_ddt_diagonal_0.x, at: time),
                    evaluateDifferential(a: 1, b: βΛ.y, c: Λ.y, g: torque_diagonal.y, y_0: θ_diagonal_0.y, y_ddt_0: θ_ddt_diagonal_0.y, at: time),
                    evaluateDifferential(a: 1, b: βΛ.z, c: Λ.z, g: torque_diagonal.z, y_0: θ_diagonal_0.z, y_ddt_0: θ_ddt_diagonal_0.z, at: time)])

                parentJoint.θ = U * θ_diagonal
            }
        }
    }

    func updateArticulatedBodies() {
        for rigidBody in rigidBodiesLevelOrder {
            updateArticulatedBody(rigidBody: rigidBody)
            for joint in rigidBody.childJoints {
                updateArticulatedBody(joint: joint, parentRigidBody: rigidBody)
            }
        }
    }

    func updateFreeBodies(at time: TimeInterval) {
        let time = Float(time)
        for rigidBody in rigidBodiesUnordered {
            updateFreeBody(rigidBody: rigidBody, at: time)
        }
    }

    func resetForces() {
        for rigidBody in rigidBodiesLevelOrder {
            rigidBody.resetForces()
        }
    }

    private func updateArticulatedBody(rigidBody: RigidBody) {
        guard let parentJoint = rigidBody.parentJoint else { return }
        let parentRigidBody = parentJoint.parentRigidBody

        rigidBody.updateTransform()

        rigidBody.angularVelocity = parentRigidBody.angularVelocity + parentJoint.rotation.act(parentJoint.θ[1])
        rigidBody.angularAcceleration = parentRigidBody.angularAcceleration + parentJoint.rotation.act(parentJoint.θ[2]) + parentRigidBody.angularVelocity.crossMatrix * rigidBody.angularVelocity

        rigidBody.velocity = parentRigidBody.velocity
        rigidBody.velocity += parentRigidBody.angularVelocity.crossMatrix * parentRigidBody.rotation.act(parentJoint.translation_local)
        rigidBody.velocity -= rigidBody.angularVelocity.crossMatrix * rigidBody.rotation.act(-rigidBody.centerOfMass_local)
        rigidBody.acceleration = parentJoint.acceleration - (rigidBody.angularAcceleration.crossMatrix + sqr(rigidBody.angularVelocity.crossMatrix)) * rigidBody.rotation.act(-rigidBody.centerOfMass_local)
    }

    private func updateArticulatedBody(joint: Joint, parentRigidBody: RigidBody) {
        joint.updateTransform()

        joint.acceleration = parentRigidBody.acceleration +
            (parentRigidBody.angularAcceleration.crossMatrix + sqr(parentRigidBody.angularVelocity.crossMatrix)) * parentRigidBody.rotation.act(joint.translation_local)

        assert(joint.isFinite)
    }

    private func updateFreeBody(rigidBody: RigidBody, at time: Float) {
        guard rigidBody.kind != .static else { return } // FIXME use composite for force?
        guard rigidBody.parentJoint == nil else { return }

        rigidBody.acceleration = rigidBody.force / rigidBody.mass
        rigidBody.angularMomentum = rigidBody.angularMomentum + time * rigidBody.torque

        rigidBody.velocity = rigidBody.velocity + time * rigidBody.acceleration
        rigidBody.angularVelocity = rigidBody.inertiaTensor.inverse * rigidBody.angularMomentum

        rigidBody.translation = rigidBody.translation + time * rigidBody.velocity
        let angularVelocityQuat = simd_quatf(real: 0, imag: rigidBody.angularVelocity)
        rigidBody.rotation = rigidBody.rotation + time/2 * angularVelocityQuat * rigidBody.rotation
        rigidBody.rotation = rigidBody.rotation.normalized

        rigidBody.inertiaTensor = float3x3(rigidBody.rotation) * rigidBody.inertiaTensor_local * float3x3(rigidBody.rotation).transpose

        rigidBody.node.simdPosition = rigidBody.translation
        rigidBody.node.simdOrientation = rigidBody.rotation

        assert(rigidBody.isFinite)
    }
}
