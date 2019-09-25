import Foundation
import simd
import SceneKit

fileprivate var i = 0

public enum Kind {
    case `static`
    case `dynamic`
}

public class RigidBody {
    let name: String
    var kind: Kind = .dynamic

    let mass: Float
    var inertiaTensor: float3x3
    let inertiaTensor_local: float3x3
    let centerOfMass_local: float3

    var force: float3 = float3.zero
    var torque: float3 = float3.zero

    // FIXME /position/
    var rotation: simd_quatf = simd_quatf.identity
    var translation: float3 = float3.zero
    // FIXME is this necessary?

    var centerOfMass: float3 = float3.zero
    var angularVelocity: float3 = float3.zero
    var angularAcceleration: float3 = float3.zero
    var angularMomentum: float3 = float3.zero // FIXME acc and momentum are used Either/or
    var velocity: float3 = float3.zero
    var acceleration: float3 = float3.zero

    var node: SCNNode

    public class func `static`() -> RigidBody {
        let rigidBody = RigidBody(mass: 0, inertiaTensor: float3x3(0), centerOfMass: float3.zero, node: SCNNode())
        rigidBody.kind = .static
        return rigidBody
    }

    init(mass: Float, inertiaTensor: float3x3, centerOfMass: float3, node: SCNNode) {
        self.name = "RigidBody[\(i)]"
        i += 1

        self.mass = mass
        self.inertiaTensor_local = inertiaTensor
        self.inertiaTensor = inertiaTensor_local
        self.centerOfMass_local = centerOfMass
        self.node = node
    }

    func apply(force: float3, torque: float3? = nil) {
        // FIXME: This torque seems wrong
        let torque = torque ?? cross(rotation.act(centerOfMass_local), force)
        self.force += force
        self.torque += torque
    }

    func resetForces() {
        self.force = float3.zero
        self.torque = float3.zero
    }

    // MARK: Articulated FIXME separate class

    weak var parentJoint: Joint? = nil
    var childJoints: [Joint] = []  // FIXME make set
    let composite = CompositeBody()

    func add(_ child: RigidBody, rotation: simd_quatf, position: float3) -> Joint {
        let joint = Joint(parent: self, child: child, rotation: rotation, position: position)
        childJoints.append(joint)
        child.parentJoint = joint
        child.updateTransform()
        return joint
    }

    func updateTransform() {
        guard let parentJoint = parentJoint else { return }

        let sora = parentJoint.θ[0]
        let rotation_local = simd_length(sora) < 10e-10 ? simd_quatf.identity : simd_quatf(angle: simd_length(sora), axis: normalize(sora))

        self.rotation = (parentJoint.rotation * rotation_local).normalized
        self.translation = parentJoint.translation

        self.inertiaTensor = float3x3(rotation) * inertiaTensor_local * float3x3(rotation).transpose

        self.centerOfMass = translation + rotation.act(centerOfMass_local)

        node.simdPosition = self.translation
        node.simdOrientation = self.rotation
    }

    func removeFromParent() {
        guard let parentJoint = parentJoint else { return }
        let parentRigidBody = parentJoint.parentRigidBody

        self.parentJoint = nil
        parentRigidBody.childJoints.removeAll { (joint: Joint) -> Bool in
            return joint === parentJoint
        }
    }

}

// FIXME think of isolating the articulatedness
extension RigidBody {
    var isRoot: Bool {
        return parentJoint == nil
    }

    var isLeaf: Bool {
        return childJoints.isEmpty
    }

    var hasOneChild: Bool {
        return childJoints.count == 1
    }

    var parentRigidBody: RigidBody? {
        return parentJoint?.parentRigidBody
    }

    func flattened() -> [RigidBody] {
        var result: [RigidBody] = []
        var queue: [RigidBody] = [self]
        searchBreadthFirst(queue: &queue, result: &result)
        return result
    }

    var leaves: [RigidBody] {
        var result: [RigidBody] = []
        for childJoint in childJoints {
            let childRigidBody = childJoint.childRigidBody
            if childRigidBody.isLeaf {
                result.append(childRigidBody)
            } else {
                result.append(contentsOf: childRigidBody.leaves)
            }
        }
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

extension RigidBody {
    var isFinite: Bool {
        return translation.isFinite &&
            rotation.isFinite &&
            inertiaTensor.isFinite &&
            angularVelocity.isFinite &&
            angularAcceleration.isFinite &&
            velocity.isFinite &&
            acceleration.isFinite &&
            centerOfMass.isFinite
    }
}

// MARK: Flattening & Leveling

// FIXME no longer necessary
class HashRigidBody: Hashable {
    let underlying: RigidBody

    init(_ underlying: RigidBody) {
        self.underlying = underlying
    }

    static func ==(lhs: HashRigidBody, rhs: HashRigidBody) -> Bool {
        return lhs.underlying === rhs.underlying
    }

    func hash(into hasher: inout Hasher) {
        hasher.combine(ObjectIdentifier(underlying))
    }
}

class RigidBodyDict<Value> {
    var dict: [HashRigidBody:Value] = [:]

    subscript(index: RigidBody) -> Value? {
        get {
            return dict[HashRigidBody(index)]
        }
        set(newValue) {
            dict[HashRigidBody(index)] = newValue
        }
    }
}

struct UnitOfWork {
    let rigidBody: RigidBody
    let climbers: [RigidBody]
}
typealias Level = [UnitOfWork]

extension RigidBody {
    func levels() -> [Level] {
        var result: [Level] = []
        var visited = Set<HashRigidBody>()

        var remaining = self.leaves
        repeat {
            var level: Level = []
            var nextRemaining: [RigidBody] = []
            while var n = remaining.popLast() {
                if n.childJoints.allSatisfy({ visited.contains(HashRigidBody($0.childRigidBody)) }) && !visited.contains(HashRigidBody(n)) {
                    var climbers: [RigidBody] = []
                    let beforeClimb = n
                    while let parentRigidBody = n.parentRigidBody, parentRigidBody.hasOneChild {
                        n = parentRigidBody
                        if !visited.contains(HashRigidBody(n)) {
                            visited.insert(HashRigidBody(n))
                            if !n.isRoot {
                                climbers.append(n)
                            }
                        }
                    }
                    if !beforeClimb.isRoot {
                        level.append(
                            UnitOfWork(rigidBody: beforeClimb, climbers: climbers))
                    }
                    if let parentJoint = n.parentJoint {
                        nextRemaining.append(parentJoint.parentRigidBody)
                    }
                }
            }
            if !level.isEmpty {
                result.append(level)
            }
            let beforeClimbs = level.map { HashRigidBody($0.rigidBody) }
            visited.formUnion(beforeClimbs)
            remaining = Array(Set(nextRemaining.map { HashRigidBody($0) })).map { $0.underlying }
        } while !remaining.isEmpty
        return result
    }
}
