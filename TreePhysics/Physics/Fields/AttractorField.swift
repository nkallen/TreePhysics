import Foundation
import ShaderTypes
import simd

public final class AttractorField: PhysicsField, PhysicsFieldStructConvertible {
    public var position: float3 = float3.zero
    public let halfExtent: float3? = float3(1, 1, 1)

    let a: Float = 0.05
    let b: Float = 0.01
    let c: Float = 0.1

    public init() {}

    public func force(rigidBody: RigidBody, time: TimeInterval) -> float3 {
        let delta = self.position - rigidBody.centerOfMass
        let distance = length(delta)
        if (distance > 0) {
            let direction = normalize(delta)
            // NOTE: this is a bell-shaped curve, so objects very far and very near are weakly affected
            let force = direction * a * powf(.e, -sqr(distance - b)/(2*c))
            return force
        } else {
            return float3(repeating: 0)
        }
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

