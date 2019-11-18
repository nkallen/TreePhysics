import Foundation
import simd
import SceneKit

public final class Joint {
    unowned let parentRigidBody: ArticulatedRigidBody
    public let childRigidBody: ArticulatedRigidBody

    let localOrientation: simd_quatf
    let localPosition: SIMD3<Float>
    var position: SIMD3<Float>
    var orientation: simd_quatf
    var inverseOrientation: simd_quatf
    var acceleration: SIMD3<Float>    // NOTE: θ[0] is the xyz orientation of the joint; θ[1] is the angular velocity, etc.
    var θ: float3x3

    var stiffness: Float
    var torqueThreshold: Float
    var damping: Float

    init(parent: ArticulatedRigidBody, child: ArticulatedRigidBody, localOrientation: simd_quatf, localPosition: SIMD3<Float>) {
        self.parentRigidBody = parent
        self.childRigidBody = child

        self.localOrientation = localOrientation
        self.localPosition = localPosition

        self.position = localPosition
        self.orientation = localOrientation
        self.inverseOrientation = localOrientation.inverse.normalized
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
        self.orientation = (parentRigidBody.orientation * localOrientation).normalized
        self.position = parentRigidBody.pivot + parentRigidBody.orientation.act(localPosition)
        self.inverseOrientation = orientation.inverse.normalized

        assert(orientation.isFinite)
        assert(position.isFinite)
        assert(inverseOrientation.isFinite)

        self.acceleration = parentRigidBody.acceleration +
            (parentRigidBody.angularAcceleration.skew + sqr(parentRigidBody.angularVelocity.skew)) * parentRigidBody.orientation.act(localPosition)
    }

    func rotate(tensor: float3x3) -> float3x3 {
        return float3x3(inverseOrientation) * tensor * float3x3(inverseOrientation).transpose
    }

    func rotate(vector: SIMD3<Float>) -> SIMD3<Float> {
        return inverseOrientation.act(vector)
    }

    var isFinite: Bool {
        return orientation.isFinite && position.isFinite && acceleration.isFinite
    }
}
