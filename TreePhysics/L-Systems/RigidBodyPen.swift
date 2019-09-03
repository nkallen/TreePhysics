import Foundation
import simd

final class RigidBodyPen: Pen {
    typealias T = RigidBody

    private var parentBranch: RigidBody
    private var start: float3? = nil

    init(parent: RigidBody) {
        self.parentBranch = parent
    }

    func start(at: float3, thickness: Float) {
        start = at
    }

    func cont(distance: Float, tangent: float3, thickness: Float) -> RigidBody {
        guard let start = start else { fatalError() }

        let newBranch = RigidBody(length: distance, radius: sqrt(thickness / .pi), density: 750)

        let parentTangent = parentBranch.translation + parentBranch.rotation.act(float3(0,1,0)) - parentBranch.position
        let rotation = simd_quatf(from: parentTangent, to: tangent).normalized

        _ = parentBranch.add(newBranch, at: rotation)

        self.start = start + distance * tangent
        self.parentBranch = newBranch

        return newBranch
    }

    var branch: RigidBodyPen {
        return RigidBodyPen(parent: parentBranch)
    }
}
