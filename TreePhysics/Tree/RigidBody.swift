import Foundation
import simd
import SceneKit

// FIXME remove
public enum Kind {
    case `static`
    case `dynamic`
}

public protocol RigidBody: class {
    var kind: Kind { get } // FIXME remove

    var parentJoint: Joint? { get set }
    var childJoints: [Joint] { get }
    var composite: CompositeBody { get }

    var mass: Float { get }
    var inertiaTensor: float3x3 { get }

    var force: float3 { get }
    var torque: float3 { get }

    // FIXME /position/
    var translation: float3 { get }
    var rotation: simd_quatf { get }

    var centerOfMass: float3 { get }
    var angularVelocity: float3 { get }
    var angularAcceleration: float3 { get }
    var velocity: float3 { get }
    var acceleration: float3 { get }

    func apply(force: float3, torque: float3?)
    func updateTransform()
    func resetForces()

    var node: SCNNode { get }
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
