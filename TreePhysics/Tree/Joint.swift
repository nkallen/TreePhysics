import Foundation
import simd
import SceneKit

final class Joint: HasTransform {
    unowned let parentRigidBody: RigidBody
    let childRigidBody: RigidBody

    var transform: matrix_float4x4 = matrix_identity_float4x4
    var angularAcceleration: Float = 0
    var angularVelocity: Float = 0
    var angle: Float = 0 {
        didSet {
            self.transform = parentRigidBody.transform * matrix4x4_translation(0, parentRigidBody.length, 0) * matrix4x4_rotation(radians: self.angle, axis: .z)
        }
    }

    let k: Float

    init(parent: RigidBody, child: RigidBody, k: Float? = nil) {
        self.parentRigidBody = parent
        self.childRigidBody = child
        self.k = k ?? Joint.computeK(radius: parent.radius)
        updateTransform()
    }

    func updateTransform() {
        self.transform = parentRigidBody.transform * matrix4x4_translation(0, parentRigidBody.length, 0) * matrix4x4_rotation(radians: self.angle, axis: .z)
    }

    private static func computeK(radius: Float) -> Float {
        return Tree.K + Tree.K * 3e5*radius
    }
}
