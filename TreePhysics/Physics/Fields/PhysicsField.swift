import Foundation
import simd
import SceneKit
import ShaderTypes

public class PhysicsField {
    var position: float3 = float3.zero
    var halfExtent: float3 = float3(.infinity, .infinity, .infinity)

    // FIXME merge these two fns
    func force(rigidBody: RigidBody, time: TimeInterval) -> float3 {
        return float3.zero
    }

    func torque(rigidBody: RigidBody, time: TimeInterval) -> float3 {
        return float3.zero
    }

    final func applies(to position: float3) -> Bool {
        return position.in(min: self.position - halfExtent, max: self.position + halfExtent)
    }

    final func apply(rigidBody: RigidBody, time: TimeInterval) {
        let force = self.force(rigidBody: rigidBody, time: time)
        let torque = self.torque(rigidBody: rigidBody, time: time)
        rigidBody.apply(force: force, torque: torque)
    }
}

public protocol PhysicsFieldStructConvertible {
    var `struct`: PhysicsFieldStruct { get }
}
