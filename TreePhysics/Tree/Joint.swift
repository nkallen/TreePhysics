import Foundation
import simd
import SceneKit

public final class Joint {
    unowned let parentRigidBody: ArticulatedRigidBody
    public let childRigidBody: ArticulatedRigidBody

    let localRotation: simd_quatf
    let localPosition: float3

    var position: float3
    var rotation: simd_quatf
    var inverseRotation: simd_quatf
    var acceleration: float3
    // NOTE: θ[0] is the xyz rotation of the joint; θ[1] is the angular velocity, etc.
    var θ: float3x3

    let stiffness: Float
    let torqueThreshold: Float

    init(parent: ArticulatedRigidBody, child: ArticulatedRigidBody, localRotation: simd_quatf, localPosition: float3) {
        self.parentRigidBody = parent
        self.childRigidBody = child

        self.localRotation = localRotation
        self.localPosition = localPosition

        self.position = localPosition
        self.rotation = localRotation
        self.inverseRotation = localRotation.inverse.normalized
        self.acceleration = float3.zero
        self.θ = float3x3(0)

        // FIXME
        switch child {
        case is Leaf:
            self.stiffness = 0.1
            self.torqueThreshold = 0.30
        default:
            self.stiffness = Internode.K
            self.torqueThreshold = Float.infinity
        }
    }

    func updateTransform() {
        self.rotation = (parentRigidBody.rotation * localRotation).normalized
        self.position = parentRigidBody.pivot + parentRigidBody.rotation.act(localPosition)
        self.inverseRotation = rotation.inverse.normalized

        self.acceleration = parentRigidBody.acceleration +
            (parentRigidBody.angularAcceleration.skew + sqr(parentRigidBody.angularVelocity.skew)) * parentRigidBody.rotation.act(localPosition)
    }

    func rotate(tensor: float3x3) -> float3x3 {
        return float3x3(inverseRotation) * tensor * float3x3(inverseRotation).transpose
    }

    func rotate(vector: float3) -> float3 {
        return inverseRotation.act(vector)
    }

    private static func computeK(radius: Float, length: Float) -> Float {
        return Internode.K
    }

    var isFinite: Bool {
        return rotation.isFinite && position.isFinite && acceleration.isFinite
    }
}
