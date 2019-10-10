import Foundation
import simd
import ShaderTypes

public final class GravityField: PhysicsField {
    let g: SIMD3<Float>
    public init(_ g: SIMD3<Float> = .zero) {
        self.g = g
    }

    override func force(rigidBody: RigidBody, time: TimeInterval) -> SIMD3<Float> {
        return g * rigidBody.mass
    }

    public func torque(rigidBody: RigidBody, time: TimeInterval) -> SIMD3<Float>? {
        return nil
    }
}
