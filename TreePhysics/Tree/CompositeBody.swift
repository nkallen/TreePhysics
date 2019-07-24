import Foundation
import simd

final class CompositeBody {
    var mass: Float = 0
    var momentOfInertia: Float = 0
    var force: float3 = float3.zero
    var torque: float3 = float3.zero
    var centerOfMass: float3 = float3.zero
}
