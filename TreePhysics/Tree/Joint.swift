import Foundation
import simd
import SceneKit

fileprivate let local_ijk = matrix_float4x4(diagonal: float4(1,1,1,0))

final class Joint: HasTransform {
    unowned let parentRigidBody: RigidBody
    let childRigidBody: RigidBody

    var transform: matrix_float4x4 = matrix_identity_float4x4 {
        didSet {
            self.rotation_world2local = matrix3x3_rotation(from: local_ijk, to: transform.inverse)
        }
    }
    var angularAcceleration: Float = 0
    var angularVelocity: Float = 0
    var angle: Float = 0 {
        didSet {
            updateTransform()
        }
    }
    var rotation_world2local: float3x3 = matrix_identity_float3x3

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

    func rotate(tensor: float3x3) -> float3x3 {
        return rotation_world2local * tensor * rotation_world2local.transpose
    }

    func rotate(vector: float3) -> float3 {
        return rotation_world2local * vector
    }

    private static func computeK(radius: Float) -> Float {
        return Tree.K + Tree.K * 4e5*radius
    }
}
