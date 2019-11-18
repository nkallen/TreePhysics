import Foundation
import simd

public final class RigidBodyPen: Pen {
    public typealias T = ArticulatedRigidBody

    private var parentBranch: ArticulatedRigidBody
    private var start: SIMD3<Float>? = nil

    public init(parent: ArticulatedRigidBody) {
        self.parentBranch = parent
    }

    public func start(at: SIMD3<Float>, orientation: simd_quatf, thickness: Float) {
        start = at
    }

    public func cont(distance: Float, orientation: simd_quatf, thickness: Float) -> ArticulatedRigidBody {
        guard let start = start else { fatalError() }

        let newBranch = Tree.internode(length: distance, radius: sqrt(thickness / .pi), density: 750)

        let worldPosition = start - parentBranch.pivot
        let localPosition = parentBranch.orientation.inverse.act(worldPosition)
        _ = parentBranch.add(newBranch, orientation: parentBranch.orientation.inverse * orientation, position: localPosition)

        self.start = start + distance * orientation.heading
        self.parentBranch = newBranch

        return newBranch
    }

    public func copy(scale: Float, orientation: simd_quatf) -> ArticulatedRigidBody {
        guard let start = start else { fatalError() }

        let newLeaf = Tree.leaf(length: scale, density: 500)

        let worldPosition = start - parentBranch.pivot
        let localPosition = parentBranch.orientation.inverse.act(worldPosition)
        let localOrientation = parentBranch.orientation.inverse * orientation
        _ = parentBranch.add(newLeaf, orientation: localOrientation, position: localPosition)

        return newLeaf
    }

    public func branch() -> RigidBodyPen {
        return RigidBodyPen(parent: parentBranch)
    }
}
