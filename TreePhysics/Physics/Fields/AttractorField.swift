import Foundation
import ShaderTypes
import simd

public final class AttractorField: PhysicsField {
    private let a: Float = 0.05
    private let b: Float = 0.01
    private let c: Float = 0.1

    public override init() {
        super.init()
        self.halfExtent = simd_float3(1, 1, 1)
    }

    override func force(rigidBody: RigidBody, time: TimeInterval) -> simd_float3 {
        let delta = self.position - rigidBody.centerOfMass
        let distance = length(delta)
        if (distance > 0) {
            let direction = normalize(delta)
            // NOTE: this is a bell-shaped curve, so objects very far and very near are weakly affected
            let force = direction * a * powf(.e, -sqr(distance - b)/(2*c))
            return force
        } else {
            return simd_float3(repeating: 0)
        }
    }

    public func torque(rigidBody: RigidBody, time: TimeInterval) -> simd_float3? {
        return nil
    }
}

extension AttractorField: PhysicsFieldStructConvertible {
    public var `struct`: PhysicsFieldStruct {
        return PhysicsFieldStruct(
            position: position,
            halfExtent: halfExtent)
    }
}
