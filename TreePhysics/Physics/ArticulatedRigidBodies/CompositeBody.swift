import Foundation
import simd

public final class CompositeBody {
    var mass: Float
    var inertiaTensor: float3x3
    var force: simd_float3
    var torque: simd_float3
    var centerOfMass: simd_float3

    init(mass: Float = 0, inertiaTensor: float3x3 = matrix_identity_float3x3, force: simd_float3 = .zero, torque: simd_float3 = .zero, centerOfMass: simd_float3 = .zero) {
        self.mass = mass
        self.inertiaTensor = inertiaTensor
        self.force = force
        self.torque = torque
        self.centerOfMass = centerOfMass
    }
}
