import Foundation
import simd
import SceneKit

public final class Joint {
    unowned let parentRigidBody: RigidBody
    let childRigidBody: RigidBody

    var rotation_local: simd_quatf = simd_quatf.identity
    var rotation: simd_quatf = simd_quatf.identity
    var translation: float3 = float3.zero
    var acceleration: float3 = float3.zero
    // NOTE: θ[0] is the xyz rotation of the joint; θ[1] is the angular velocity, etc.
    var θ: float3x3 = float3x3(0)
    var rotation_world2local: simd_quatf = simd_quatf.identity
    let translation_local: float3

    let k: Float // FIXME rename stiffness
    let torqueThreshold: Float

    init(parent: RigidBody, child: RigidBody, rotation: simd_quatf, position: float3) {
        self.parentRigidBody = parent
        self.childRigidBody = child
        self.rotation_local = rotation
        switch child {
        case is Leaf:
            self.k = 1
            self.torqueThreshold = 0.4
        default:
            self.k = Internode.K
            self.torqueThreshold = Float.infinity
        }
        self.translation_local = position
        updateTransform()
    }

    // FIXME pivot/position
    var position: float3 {
        return translation
    }

    func updateTransform() {
        self.rotation = (parentRigidBody.rotation * rotation_local).normalized
        self.translation = parentRigidBody.translation + parentRigidBody.rotation.act(translation_local)
        self.rotation_world2local = rotation.inverse.normalized
    }

    func rotate(tensor: float3x3) -> float3x3 {
        return float3x3(rotation_world2local) * tensor * float3x3(rotation_world2local).transpose
    }

    func rotate(vector: float3) -> float3 {
        return rotation_world2local.act(vector)
    }

    private static func computeK(radius: Float, length: Float) -> Float {
        return Internode.K
    }

    var isFinite: Bool {
        return rotation.isFinite && translation.isFinite && acceleration.isFinite
    }
}
