import Foundation
import simd
import ShaderTypes

public final class GravityField: PhysicsField {
    public var g: float3
    public let position: float3 = float3.zero
    public let halfExtent: float3? = nil

    public init(_ g: float3 = float3.zero) {
        self.g = g
    }

    public func force(rigidBody: RigidBody, time: TimeInterval) -> float3 {
        return g * rigidBody.mass
    }

    public var `struct`: PhysicsFieldStruct {
        return PhysicsFieldStruct(
            position: position,
            halfExtent: halfExtent ?? float3(repeating: -1))
    }

    public func torque(rigidBody: RigidBody, time: TimeInterval) -> float3? {
        return nil
    }
}
