import Foundation
import simd
import SceneKit

final public class ArticulatedRigidBody: RigidBody {
    weak var parentJoint: Joint? = nil
    public var childJoints: Set<Joint> = []
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

    func add(_ child: ArticulatedRigidBody, rotation: simd_quatf = simd_quatf(angle: -.pi/4, axis: .z), position: SIMD3<Float> = .zero) -> Joint {
        let joint = Joint(parent: self, child: child, localRotation: rotation, localPosition: position)
        childJoints.insert(joint)
        child.parentJoint = joint
        joint.updateTransform()
        child.updateTransform()
        return joint
    }

    override func apply(force: SIMD3<Float>, torque: SIMD3<Float> = .zero) {
        var torque = torque
        if parentJoint != nil {
            torque += cross(rotation.act(-localPivot), force)
        }

        super.apply(force: force, torque: torque)
    }

    override func updateTransform() {
        if let parentJoint = parentJoint {
            let sora = parentJoint.θ[0]
            assert(sora.isFinite)
            let localRotation = simd_length(sora) < 10e-10 ? simd_quatf.identity : simd_quatf(angle: simd_length(sora), axis: normalize(sora))

            self.rotation = (parentJoint.rotation * localRotation).normalized
            assert(parentJoint.rotation.isFinite)
            assert(localRotation.isFinite)
            assert(rotation.isFinite)
            self.pivot = parentJoint.position
            assert(pivot.isFinite)

            self.inertiaTensor = float3x3(rotation) * localInertiaTensor * float3x3(rotation).transpose
            assert(inertiaTensor.isFinite)
            
            self.centerOfMass = self.pivot + rotation.act(-localPivot)
            assert(centerOfMass.isFinite)
        }

        self.pivot = centerOfMass + rotation.act(localPivot)

        super.updateTransform()
    }

    func removeFromParent() {
        guard let parentJoint = parentJoint else { return }
        let parentRigidBody = parentJoint.parentRigidBody

        self.parentJoint = nil
        parentRigidBody.childJoints.remove(parentJoint)
    }

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

    var leaves: [ArticulatedRigidBody] {
        var result: [ArticulatedRigidBody] = []
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

    func levels() -> [Level] {
        var result: [Level] = []
        var visited = Set<ArticulatedRigidBody>()

        var remaining = self.leaves
        repeat {
            var level: Level = []
            var nextRemaining: [ArticulatedRigidBody] = []
            while var n = remaining.popLast() {
                if n.childJoints.allSatisfy({ visited.contains($0.childRigidBody) }) && !visited.contains(n) {
                    var climbers: [ArticulatedRigidBody] = []
                    let beforeClimb = n
                    while let parentRigidBody = n.parentRigidBody, parentRigidBody.hasOneChild {
                        n = parentRigidBody
                        if !visited.contains(n) {
                            visited.insert(n)
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
            let beforeClimbs = level.map { $0.rigidBody }
            visited.formUnion(beforeClimbs)
            remaining = Array(Set(nextRemaining))
        } while !remaining.isEmpty
        return result
    }

    override var isFinite: Bool {
        return super.isFinite && pivot.isFinite
    }
}

struct UnitOfWork {
    let rigidBody: ArticulatedRigidBody
    let climbers: [ArticulatedRigidBody]
}
typealias Level = [UnitOfWork]
