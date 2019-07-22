import Foundation
import simd

protocol Pen {
    func start(at: float2, tangent: float2, thickness: Float)
    func cont(to: float2, tangent: float2, thickness: Float)
    func end(at: float2, tangent: float2, thickness: Float)
    var branch: Pen { get }
}

class CylinderPen: Pen {
    private var parentBranch: RigidBody
    private var parentAngle: Float
    private var start: (float2, float2)? = nil

    init(parent: RigidBody, angle: Float = .pi/2) {
        self.parentBranch = parent
        self.parentAngle = angle
    }

    func start(at: float2, tangent: float2, thickness: Float) {
        start = (at, tangent)
    }

    func cont(to: float2, tangent: float2, thickness: Float) {
        guard let start = start else { fatalError() }

        let (startPosition, _) = start
        let delta = to - startPosition
        let newBranch = RigidBody(length: distance(startPosition, to), radius: sqrt(thickness / .pi), density: 750)

        let angle = atan2(delta.y, delta.x)
        parentBranch.add(newBranch, at: angle - parentAngle)
        self.parentAngle = angle

        self.start = (to, tangent)
        self.parentBranch = newBranch
    }

    func end(at: float2, tangent: float2, thickness: Float) {
        cont(to: at, tangent: tangent, thickness: thickness)
    }

    var branch: Pen {
        return CylinderPen(parent: parentBranch, angle: parentAngle)
    }
}
