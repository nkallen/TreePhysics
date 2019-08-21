import Foundation
import simd

protocol PhysicsField {
    var position: float3 { get }
    var halfExtent: float3? { get }
    var `struct`: PhysicsFieldStruct { get }
    func eval(position: float3, velocity: float3, mass: Float, time: TimeInterval) -> float3
}

extension PhysicsField {
    func applies(to position: float3) -> Bool {
        guard let halfExtent = halfExtent else { return true }

        return position >= self.position - halfExtent && position <= self.position + halfExtent
    }
}

final class GravityField: PhysicsField {
    var g: float3
    let position: float3 = float3.zero
    let halfExtent: float3? = nil

    init(_ g: float3 = float3.zero) {
        self.g = g
    }

    func eval(position: float3, velocity: float3, mass: Float, time: TimeInterval) -> float3 {
        return g * mass
    }

    var `struct`: PhysicsFieldStruct {
        return PhysicsFieldStruct(
            position: position,
            halfExtent: halfExtent ?? float3(repeating: -1))
    }
}

final class AttractorField: PhysicsField {
    var position: float3 = float3.zero
    var halfExtent: float3? = float3(0.1, 0.1, 0.1)

    let a: Float = 0.05
    let b: Float = 0.01
    let c: Float = 0.1

    func eval(position: float3, velocity: float3, mass: Float, time: TimeInterval) -> float3 {
        let delta = self.position - position
        let distance = length(delta)
        let direction = normalize(delta)
        // NOTE: this is a bell-shaped curve, so objects very far and very near are weakly affected
        return direction * a * powf(.e, -sqr(distance - b)/(2*c))
    }

    var `struct`: PhysicsFieldStruct {
        return PhysicsFieldStruct(
            position: position,
            halfExtent: halfExtent ?? float3(repeating: -1))
    }
}
