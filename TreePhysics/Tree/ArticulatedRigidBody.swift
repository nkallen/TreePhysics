import Foundation
import simd
import SceneKit

public class ArticulatedRigidBody: RigidBody {
    weak var parentJoint: Joint? = nil
    var childJoints: [Joint] = []  // FIXME make set
    let composite = CompositeBody()

    public class func `static`() -> ArticulatedRigidBody {
        let rigidBody = ArticulatedRigidBody(mass: 0, localInertiaTensor: float3x3(0), localPivot: float3.zero, node: SCNNode())
        rigidBody.kind = .static
        return rigidBody
    }

    public class func dynamic() -> ArticulatedRigidBody {
        let rigidBody = ArticulatedRigidBody(mass: 0, localInertiaTensor: float3x3(0), localPivot: float3.zero, node: SCNNode())
        rigidBody.kind = .dynamic
        return rigidBody
    }

    func add(_ child: ArticulatedRigidBody, rotation: simd_quatf, position: float3) -> Joint {
        let joint = Joint(parent: self, child: child, rotation: rotation, position: position)
        childJoints.append(joint)
        child.parentJoint = joint
        child.updateTransform()
        return joint
    }

    override func updateTransform() {
        if let parentJoint = parentJoint {
            let sora = parentJoint.θ[0]
            let rotation_local = simd_length(sora) < 10e-10 ? simd_quatf.identity : simd_quatf(angle: simd_length(sora), axis: normalize(sora))
            
            self.rotation = (parentJoint.rotation * rotation_local).normalized
            self.pivot = parentJoint.position
            
            self.inertiaTensor = float3x3(rotation) * localInertiaTensor * float3x3(rotation).transpose
            
            self.centerOfMass = self.pivot + rotation.act(-localPivot)
        }

        super.updateTransform()
    }

    func removeFromParent() {
        guard let parentJoint = parentJoint else { return }
        let parentRigidBody = parentJoint.parentRigidBody

        self.parentJoint = nil
        parentRigidBody.childJoints.removeAll { (joint: Joint) -> Bool in
            return joint === parentJoint
        }
    }

    var isRoot: Bool {
        return parentJoint == nil
    }

    var isLeaf: Bool {
        return childJoints.isEmpty
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
        searchBreadthFirst(queue: &queue, result: &result)
        return result
    }

    private func searchBreadthFirst(queue: inout [ArticulatedRigidBody], result: inout [ArticulatedRigidBody]) {
        while !queue.isEmpty {
            let start = queue.removeFirst()
            result.append(start)
            for childJoint in start.childJoints {
                queue.append(childJoint.childRigidBody)
            }
        }
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

}

struct UnitOfWork {
    let rigidBody: ArticulatedRigidBody
    let climbers: [ArticulatedRigidBody]
}
typealias Level = [UnitOfWork]
