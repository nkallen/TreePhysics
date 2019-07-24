import Foundation
import simd

final class RigidBodyPen: Pen {
    typealias T = RigidBody

    private var parentBranch: RigidBody
    private var parentAngle: Float
    private var start: float3? = nil

    init(parent: RigidBody, angle: Float = .pi/2) {
        self.parentBranch = parent
        self.parentAngle = angle
    }

    func start(at: float3, thickness: Float) {
        start = at
    }

    func cont(distance: Float, tangent: float3, thickness: Float) -> RigidBody {
        guard let start = start else { fatalError() }

        let newBranch = RigidBody(length: distance, radius: sqrt(thickness / .pi), density: 750)

        let angle = atan2(tangent.y, tangent.x)
        parentBranch.add(newBranch, at: angle - parentAngle)
        self.parentAngle = angle

        self.start = start + distance * tangent
        self.parentBranch = newBranch

        return newBranch
    }

    var branch: RigidBodyPen {
        return RigidBodyPen(parent: parentBranch, angle: parentAngle)
    }
}
