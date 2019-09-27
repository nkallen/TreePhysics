import Foundation
import simd
import SceneKit

// FIXME audit / rename

public final class Joint {
    unowned let parentRigidBody: ArticulatedRigidBody
    public let childRigidBody: ArticulatedRigidBody

    var localRotation: simd_quatf = simd_quatf.identity
    public var rotation: simd_quatf = simd_quatf.identity
    var position: float3 = float3.zero
    var acceleration: float3 = float3.zero
    // NOTE: θ[0] is the xyz rotation of the joint; θ[1] is the angular velocity, etc.
    var θ: float3x3 = float3x3(0)
    var rotation_world2local: simd_quatf = simd_quatf.identity
    let localPosition: float3

    let stiffness: Float
    let torqueThreshold: Float

    init(parent: ArticulatedRigidBody, child: ArticulatedRigidBody, rotation: simd_quatf, position: float3) {
        self.parentRigidBody = parent
        self.childRigidBody = child
        self.localRotation = rotation
        switch child {
        case is Leaf:
            self.stiffness = 0.1
            self.torqueThreshold = 0.30
        default:
            self.stiffness = Internode.K
            self.torqueThreshold = Float.infinity
        }
        self.localPosition = position
        updateTransform()
    }

    func updateTransform() {
        self.rotation = (parentRigidBody.rotation * localRotation).normalized
        self.position = parentRigidBody.pivot + parentRigidBody.rotation.act(localPosition)
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
        return rotation.isFinite && position.isFinite && acceleration.isFinite
    }
}
