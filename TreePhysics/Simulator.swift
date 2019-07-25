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

            composite.momentOfInertia = rigidBody.momentOfInertia + rigidBody.mass * square(distance(rigidBody.centerOfMass, composite.centerOfMass))


            let p = composite.centerOfMass - rigidBody.centerOfMass
            let pstar = matrix3x3_cross(p)
            composite.inertiaTensor = rigidBody.inertiaTensor - rigidBody.mass * matrix_multiply(pstar, pstar)


            for childJoint in rigidBody.childJoints {
                let childRigidBody = childJoint.childRigidBody
                let childComposite = childRigidBody.composite

                // using the parallel axis theorem I' = I + md^2, calculate inertia of this body about the
                // center of mass of the composite body, then add the child inertia's (also relative to the
                // center of mass of the composite). NOTE: This is in a separate loop because of the
                // dependency on the composite.centerOfMass /= composite.mass step
                composite.momentOfInertia += childComposite.momentOfInertia +
                    childComposite.mass * square(distance(composite.centerOfMass, childComposite.centerOfMass))


                let p = composite.centerOfMass - childComposite.centerOfMass
                let pstar = matrix3x3_cross(p)

                composite.inertiaTensor += childComposite.inertiaTensor +
                    childComposite.mass * matrix_multiply(pstar, pstar)
            }
        }
    }

    func updateSprings(at time: TimeInterval) {
        for rigidBody in rigidBodiesLevelOrder { // Order does not matter
            if let parentJoint = rigidBody.parentJoint {
                let compositeInertiaRelativeToJoint = rigidBody.composite.momentOfInertia +
                    rigidBody.composite.mass * square(distance(rigidBody.composite.centerOfMass, parentJoint.position))

                // Solve: Iθ'' + (αI + βK)θ' + Kθ = τ
                // θ(0) = joint's angle, θ'(0) = joint's angular velocity

                if parentJoint.k == .infinity {
                    parentJoint.angle = 0
                    parentJoint.angularVelocity = 0
                    parentJoint.angularAcceleration = 0
                } else {
                    let solution = solve_differential(a: Double(compositeInertiaRelativeToJoint), b: Double(Tree.B * parentJoint.k), c: Double(parentJoint.k), g: Double(rigidBody.composite.torque.z), y_0: Double(parentJoint.angle), y_ddt_0: Double(parentJoint.angularVelocity))
                    let thetas = evaluate(differential: solution, at: time)
                    parentJoint.angle = max(Tree.minAngle, min(Tree.maxAngle, thetas.x))
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
