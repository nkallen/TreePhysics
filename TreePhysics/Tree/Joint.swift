import Foundation
import simd
import SceneKit

final class Joint {
    unowned let parentRigidBody: RigidBody
    let childRigidBody: RigidBody

    var rotation_local: float3x3 = matrix_identity_float3x3
    var rotation: float3x3 = matrix_identity_float3x3
    var translation: float3 = float3.zero
    var acceleration: float3 = float3.zero
    // NOTE: θ[0] is the xyz rotation of the joint; θ[1] is the angular velocity, etc.
    var θ: float3x3 = float3x3(0)
    private(set) var rotation_world2local: float3x3 = matrix_identity_float3x3
    let translation_local: float3
    let position_local: float3

    let k: Float

    init(parent: RigidBody, child: RigidBody, at rotation: float3x3) {
        self.parentRigidBody = parent
        self.childRigidBody = child
        self.rotation_local = rotation
        self.k = Joint.computeK(radius: parent.radius, length: parent.length)
        self.translation_local = float3(0, parentRigidBody.length, 0)
        self.position_local = float3(0, parentRigidBody.length, 0) // FIXME weird duplicate
        updateTransform()
    }

    var position: float3 {
        return translation
    }

    func updateTransform() {
        self.rotation = parentRigidBody.rotation * rotation_local
        self.translation = parentRigidBody.translation + parentRigidBody.rotation * translation_local
        self.rotation_world2local = rotation.transpose

        self.acceleration = parentRigidBody.acceleration +
            (parentRigidBody.angularAcceleration.crossMatrix + sqr(parentRigidBody.angularVelocity.crossMatrix)) * parentRigidBody.rotation * position_local
    }

    func rotate(tensor: float3x3) -> float3x3 {
        return rotation_world2local * tensor * rotation_world2local.transpose
    }

    func rotate(vector: float3) -> float3 {
        return rotation_world2local * vector
    }

    private static func computeK(radius: Float, length: Float) -> Float {
        return RigidBody.K
    }
}
