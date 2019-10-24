import Foundation
import simd

public final class CompositeBody {
    var mass: Float
    var inertiaTensor: float3x3
    var force: SIMD3<Float>
    var torque: SIMD3<Float>
    var centerOfMass: SIMD3<Float>

    init(mass: Float = 0, inertiaTensor: float3x3 = matrix_identity_float3x3, force: SIMD3<Float> = .zero, torque: SIMD3<Float> = .zero, centerOfMass: SIMD3<Float> = .zero) {
        self.mass = mass
        self.inertiaTensor = inertiaTensor
        self.force = force
        self.torque = torque
        self.centerOfMass = centerOfMass
    }
}
