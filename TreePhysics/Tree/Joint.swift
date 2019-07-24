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
        if radius <= 0.003 {
            return Tree.K
        } else if radius <= 0.005 {
            return Tree.K
        } else if radius <= 0.01 {
            return 4 * Tree.K
        } else if radius <= 0.02 {
            return 256 * Tree.K
        } else if radius <= 0.04 {
            return 256 * Tree.K
        } else if radius <= 0.075 {
            return 1024 * Tree.K
        } else if radius <= 0.15 {
            return 4096 * Tree.K
        } else {
            return 16384 * Tree.K
        }
    }
}
