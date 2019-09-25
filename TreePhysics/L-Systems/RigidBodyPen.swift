import Foundation
import simd

// FIXME: we need a unit test to ensure rigidbody pen and cylinderpen are consistent

public final class RigidBodyPen: Pen {
    public typealias T = RigidBody

    private var parentBranch: RigidBody
    private var start: float3? = nil

    public init(parent: RigidBody) {
        self.parentBranch = parent
    }

    public func start(at: float3, thickness: Float) {
        start = at
    }

    // FIXME replace tangent with orientation
    public func cont(distance: Float, tangent: float3, thickness: Float) -> RigidBody {
        guard let start = start else { fatalError() }

        let newBranch = Internode(length: distance, radius: sqrt(thickness / .pi), density: 750)

        let parentTangent = parentBranch.rotation.act(float3(0,1,0))
        let rotation = simd_quatf(from: parentTangent, to: tangent).normalized

        let worldPosition = start - parentBranch.translation
        let localPosition = parentBranch.rotation.inverse.act(worldPosition)
        _ = parentBranch.add(newBranch, rotation: rotation, position: localPosition)

        self.start = start + distance * tangent
        self.parentBranch = newBranch

        return newBranch
    }

    public func copy(scale: Float, orientation: simd_quatf) -> RigidBody {
        guard let start = start else { fatalError() }

        let newLeaf = Leaf(length: scale, density: 500)

        let worldPosition = start - parentBranch.translation
        let localPosition = parentBranch.rotation.inverse.act(worldPosition)
        let localOrientation = parentBranch.rotation.inverse * orientation
        _ = parentBranch.add(newLeaf, rotation: localOrientation, position: localPosition)

        return newLeaf
    }

    public var branch: RigidBodyPen {
        return RigidBodyPen(parent: parentBranch)
    }
}
