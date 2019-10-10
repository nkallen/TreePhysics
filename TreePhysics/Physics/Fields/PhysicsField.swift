import Foundation
import simd
import SceneKit
import ShaderTypes

public class PhysicsField {
    var position: SIMD3<Float> = .zero
    var halfExtent: SIMD3<Float> = SIMD3<Float>(.infinity, .infinity, .infinity)

    func force(rigidBody: RigidBody, time: TimeInterval) -> SIMD3<Float> {
        return .zero
    }

    func torque(rigidBody: RigidBody, time: TimeInterval) -> SIMD3<Float> {
        return .zero
    }

    final func applies(to position: SIMD3<Float>) -> Bool {
        return position.in(min: self.position - halfExtent, max: self.position + halfExtent)
    }

    final func apply(rigidBody: RigidBody, time: TimeInterval) {
        guard rigidBody.kind == .dynamic else { return }

        let force = self.force(rigidBody: rigidBody, time: time)
        let torque = self.torque(rigidBody: rigidBody, time: time)
        rigidBody.apply(force: force, torque: torque)
    }
}

public protocol PhysicsFieldStructConvertible {
    var `struct`: PhysicsFieldStruct { get }
}
