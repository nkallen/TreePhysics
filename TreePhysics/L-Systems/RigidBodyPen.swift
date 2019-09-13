import Foundation
import simd

public final class RigidBodyPen: Pen {
    public typealias T = RigidBody

    private var parentBranch: Internode
    private var start: float3? = nil

    public init(parent: Internode) {
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

        _ = parentBranch.add(newBranch, at: rotation)

        self.start = start + distance * tangent
        self.parentBranch = newBranch

        return newBranch
    }

    public func copy(scale: Float, orientation: simd_quatf) -> RigidBody {
        guard let _ = start else { fatalError() }

        let newLeaf = Leaf(length: scale, density: 750)

        _ = parentBranch.add(newLeaf, at: orientation)

        return newLeaf
    }

    public var branch: RigidBodyPen {
        return RigidBodyPen(parent: parentBranch)
    }
}
