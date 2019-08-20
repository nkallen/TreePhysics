import Foundation
import simd

final class CompositeBody {
    var mass: Float
    var inertiaTensor: float3x3
    var force: float3
    var torque: float3
    var centerOfMass: float3

    init(mass: Float = 0, inertiaTensor: float3x3 = matrix_identity_float3x3, force: float3 = float3.zero, torque: float3 = float3.zero, centerOfMass: float3 = float3.zero) {
        self.mass = mass
        self.inertiaTensor = inertiaTensor
        self.force = force
        self.torque = torque
        self.centerOfMass = centerOfMass
    }
}
