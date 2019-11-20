import Foundation
import simd
import SceneKit
import ShaderTypes

public class PhysicsField {
    public var position: simd_float3 = .zero
    public var halfExtent: simd_float3 = simd_float3(.infinity, .infinity, .infinity)

    func force(rigidBody: RigidBody, time: TimeInterval) -> simd_float3 {
        return .zero
    }

    func torque(rigidBody: RigidBody, time: TimeInterval) -> simd_float3 {
        return .zero
    }

    final func applies(to position: simd_float3) -> Bool {
        return position.in(min: self.position - halfExtent, max: self.position + halfExtent)
    }

    final func apply(rigidBody: RigidBody, time: TimeInterval) {
        guard rigidBody.kind == .dynamic else { return }

        let force = self.force(rigidBody: rigidBody, time: time)
        let torque = self.torque(rigidBody: rigidBody, time: time)
        rigidBody.apply(force: force, torque: torque)
    }
}
