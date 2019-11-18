import Foundation
import simd
import SceneKit

final public class ArticulatedRigidBody: RigidBody {
    weak var parentJoint: Joint? = nil
    public var childJoints: [Joint] = []
    let composite = CompositeBody()

    // The `pivot` is the point at which the object connects to its parent, and the "local" pivot is relative to the center of mass.
    let localPivot: SIMD3<Float>
    var pivot: SIMD3<Float>

    public class func `static`() -> ArticulatedRigidBody {
        let rigidBody = ArticulatedRigidBody(kind: .static, mass: 0, localInertiaTensor: float3x3(0), localPivot: .zero, shape: nil, node: SCNNode())
        return rigidBody
    }

    public init(kind: Kind, mass: Float, localInertiaTensor: float3x3, localPivot: SIMD3<Float>, shape: Shape?, node: SCNNode) {
        self.localPivot = localPivot
        self.pivot = localPivot

        super.init(kind: kind, mass: mass, localInertiaTensor: localInertiaTensor, shape: shape, node: node)
    }

    func add(_ child: ArticulatedRigidBody, orientation: simd_quatf = simd_quatf(angle: -.pi/4, axis: .z), position: SIMD3<Float> = .zero) -> Joint {
        let joint = Joint(parent: self, child: child, localOrientation: orientation, localPosition: position)
        childJoints.append(joint)
        child.parentJoint = joint
        joint.updateTransform()
        child.updateTransform()
        return joint
    }

    override func apply(force: SIMD3<Float>, torque: SIMD3<Float> = .zero) {
        var torque = torque
        if parentJoint != nil {
            torque += cross(orientation.act(-localPivot), force)
        }

        super.apply(force: force, torque: torque)
    }

    override func updateTransform() {
        if let parentJoint = parentJoint {
            let sora = parentJoint.θ[0]
            assert(sora.isFinite)
            let localorientation = simd_length(sora) < 10e-10 ? simd_quatf.identity : simd_quatf(angle: simd_length(sora), axis: normalize(sora))

            self.orientation = (parentJoint.orientation * localorientation).normalized
            self.pivot = parentJoint.position

            self.inertiaTensor = float3x3(orientation) * localInertiaTensor * float3x3(orientation).transpose
            self.centerOfMass = self.pivot + orientation.act(-localPivot)
        }

        self.pivot = centerOfMass + orientation.act(localPivot)

        super.updateTransform()
    }

    func removeFromParent() {
        guard let parentJoint = parentJoint else { return }
        let parentRigidBody = parentJoint.parentRigidBody

        self.parentJoint = nil
        parentRigidBody.childJoints.removeAll { $0 === self }
    }

    override var isFinite: Bool {
        return super.isFinite && pivot.isFinite
    }
}

extension ArticulatedRigidBody {
    var isRoot: Bool {
        return parentJoint == nil
    }

    var isLeaf: Bool {
        return childJoints.isEmpty
    }

    var onlyChild: ArticulatedRigidBody? {
        guard childJoints.count == 1, let childJoint = childJoints.first else { return nil }
        return childJoint.childRigidBody
    }

    var hasOneChild: Bool {
        return childJoints.count == 1
    }

    var parentRigidBody: ArticulatedRigidBody? {
        return parentJoint?.parentRigidBody
    }

    func flattened() -> [ArticulatedRigidBody] {
        var result: [ArticulatedRigidBody] = []
        var queue: [ArticulatedRigidBody] = [self]
        while !queue.isEmpty {
            let start = queue.removeFirst()
            result.append(start)
            for childJoint in start.childJoints {
                queue.append(childJoint.childRigidBody)
            }
        }
        return result
    }

    var children: [ArticulatedRigidBody] {
        return childJoints.map { $0.childRigidBody }
    }

    func levels() -> [[UnitOfWork]] {
        var result: [[UnitOfWork]] = []
        func nextLevel(_ level: [UnitOfWork]) -> [UnitOfWork] {
            var result: [UnitOfWork] = []
            for (parentId, item) in level.enumerated() {
                for (childIndex, child) in item.rigidBody.children.enumerated() {
                    var climbers: [ArticulatedRigidBody] = [child]
                    var current = child
                    while let next = current.onlyChild {
                        climbers.append(next)
                        current = next
                    }
                    result.append(UnitOfWork(
                        childCount: child.childJoints.count,
                        childIndex: childIndex,
                        parentId: parentId,
                        rigidBody: child,
                        climbers: []))
                }
            }
            return result.sorted()
        }
        var current = [
            UnitOfWork(
                childCount: childJoints.count,
                childIndex: 0,
                parentId: 0,
                rigidBody: self,
                climbers: [])]
        while true {
            result.append(current)
            let next = nextLevel(current)
            if next.isEmpty { break }
            current = next
        }
        return result
    }
}

struct UnitOfWork {
    let childCount: Int
    let childIndex: Int
    let parentId: Int
    let rigidBody: ArticulatedRigidBody
    let climbers: [ArticulatedRigidBody]
}

extension UnitOfWork: Comparable {
    static func < (lhs: UnitOfWork, rhs: UnitOfWork) -> Bool {
        return lhs.childCount < rhs.childCount &&
            lhs.childIndex < rhs.childIndex &&
            lhs.parentId < rhs.parentId
    }
}
