import Foundation
import simd
import SceneKit

public final class Joint {
    unowned let parentRigidBody: ArticulatedRigidBody
    public let childRigidBody: ArticulatedRigidBody

    let localRotation: simd_quatf
    let localPosition: SIMD3<Float>
    var position: SIMD3<Float>
    var rotation: simd_quatf
    var inverseRotation: simd_quatf
    var acceleration: SIMD3<Float>    // NOTE: θ[0] is the xyz rotation of the joint; θ[1] is the angular velocity, etc.
    var θ: float3x3

    var stiffness: Float
    var torqueThreshold: Float
    var damping: Float

    init(parent: ArticulatedRigidBody, child: ArticulatedRigidBody, localRotation: simd_quatf, localPosition: SIMD3<Float>) {
        self.parentRigidBody = parent
        self.childRigidBody = child

        self.localRotation = localRotation
        self.localPosition = localPosition

        self.position = localPosition
        self.rotation = localRotation
        self.inverseRotation = localRotation.inverse.normalized
        self.acceleration = .zero
        self.θ = float3x3(0)

        // FIXME
        switch child.shape {
        case .leaf:
            self.stiffness = 0.1
            self.torqueThreshold = 0.30
            self.damping = Tree.β
        case .internode, nil:
            self.stiffness = Tree.K
            self.torqueThreshold = Float.infinity
            self.damping = Tree.β
        }
    }

    func updateTransform() {
        self.rotation = (parentRigidBody.rotation * localRotation).normalized
        self.position = parentRigidBody.pivot + parentRigidBody.rotation.act(localPosition)
        self.inverseRotation = rotation.inverse.normalized

        assert(rotation.isFinite)
        assert(position.isFinite)
        assert(inverseRotation.isFinite)

        self.acceleration = parentRigidBody.acceleration +
            (parentRigidBody.angularAcceleration.skew + sqr(parentRigidBody.angularVelocity.skew)) * parentRigidBody.rotation.act(localPosition)
    }

    func rotate(tensor: float3x3) -> float3x3 {
        return float3x3(inverseRotation) * tensor * float3x3(inverseRotation).transpose
    }

    func rotate(vector: SIMD3<Float>) -> SIMD3<Float> {
        return inverseRotation.act(vector)
    }

    var isFinite: Bool {
        return rotation.isFinite && position.isFinite && acceleration.isFinite
    }
}
