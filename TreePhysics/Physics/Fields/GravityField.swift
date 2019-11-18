import Foundation
import simd
import ShaderTypes

public final class GravityField: PhysicsField {
    let g: simd_float3
    public init(_ g: simd_float3 = .zero) {
        self.g = g
    }

    override func force(rigidBody: RigidBody, time: TimeInterval) -> simd_float3 {
        return g * rigidBody.mass
    }

    public func torque(rigidBody: RigidBody, time: TimeInterval) -> simd_float3? {
        return nil
    }
}
