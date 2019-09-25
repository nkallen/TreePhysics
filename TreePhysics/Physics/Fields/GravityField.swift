import Foundation
import simd
import ShaderTypes

public final class GravityField: PhysicsField {
    let g: float3

    public init(_ g: float3 = float3.zero) {
        self.g = g
    }

    override func force(rigidBody: RigidBody, time: TimeInterval) -> float3 {
        return g * rigidBody.mass
    }

    public func torque(rigidBody: RigidBody, time: TimeInterval) -> float3? {
        return nil
    }
}
