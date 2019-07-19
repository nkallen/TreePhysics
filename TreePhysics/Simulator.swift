import Foundation
import simd

class Simulator {
    let tree: Tree
    let rigidBodiesLevelOrder: [RigidBody]
    let rigidBodiesReverseLevelOrder: [RigidBody]

    init(tree: Tree) {
        self.tree = tree
        self.rigidBodiesLevelOrder = tree.flatten
        self.rigidBodiesReverseLevelOrder = self.rigidBodiesLevelOrder.reversed()
    }

    func update(at time: TimeInterval) {
        updateCompositeBodies()
        updateSprings()
        updateRigidBodies()
    }

    private func updateCompositeBodies() {
        for rigidBody in rigidBodiesReverseLevelOrder {
            rigidBody.apply(force: Tree.gravity, at: 0.5) // FIXME

            rigidBody.composite.mass = rigidBody.mass + rigidBody.childJoints.map { $0.childRigidBody.composite.mass }.sum
            rigidBody.composite.force = rigidBody.force + rigidBody.childJoints.map { $0.childRigidBody.composite.force }.sum

            // this is due to distributivity of cross product
            rigidBody.composite.torque = rigidBody.torque + rigidBody.childJoints.map {
                cross($0.position - rigidBody.position, $0.childRigidBody.composite.force) + $0.childRigidBody.composite.torque
                }.sum

            rigidBody.composite.centerOfMass = (rigidBody.mass * rigidBody.centerOfMass + rigidBody.childJoints.map { $0.childRigidBody.composite.mass * $0.childRigidBody.composite.centerOfMass }.sum) / (rigidBody.mass + rigidBody.childJoints.map { $0.childRigidBody.composite.mass }.sum)

            // using the parallel axis theorem I' = I + md^2, calculate inertia of this body about the
            // center of mass of the composite body, then add the child inertia's (also relative to the
            // center of mass of the composite)
            rigidBody.composite.momentOfInertia = rigidBody.momentOfInertia +
                rigidBody.mass * square(distance(rigidBody.centerOfMass, rigidBody.composite.centerOfMass)) +
                rigidBody.childJoints.map { $0.childRigidBody.composite.momentOfInertia + $0.childRigidBody.composite.mass * square(distance(rigidBody.composite.centerOfMass, $0.childRigidBody.composite.centerOfMass)) }.sum
        }
    }

    private func updateSprings() {

    }

    private func updateRigidBodies() {

    }
}

extension Tree {
    var flatten: [RigidBody] {
        var result: [RigidBody] = []
        var queue: [RigidBody] = [root]
        searchBreadthFirst(queue: &queue, result: &result)
        return result
    }

    private func searchBreadthFirst(queue: inout [RigidBody], result: inout [RigidBody]) {
        while !queue.isEmpty {
            let start = queue.removeFirst()
            result.append(start)
            for childJoint in start.childJoints {
                queue.append(childJoint.childRigidBody)
            }
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
