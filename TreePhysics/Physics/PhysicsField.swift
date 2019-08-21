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

        return position.in(min: self.position - halfExtent, max: self.position + halfExtent)
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
    let halfExtent: float3? = float3(1, 1, 1)

    let a: Float = 0.05
    let b: Float = 0.01
    let c: Float = 0.1

    func eval(position: float3, velocity: float3, mass: Float, time: TimeInterval) -> float3 {
//        print("does apply")
        let delta = self.position - position
        let distance = length(delta)
//        print("delta:", delta)
//        print("distance:", distance)
        if (distance > 0) {
            let direction = normalize(delta)
//            print("direction:", direction)
            // NOTE: this is a bell-shaped curve, so objects very far and very near are weakly affected
            let force = direction * a * powf(.e, -sqr(distance - b)/(2*c))
//            print("force:", force)
            return force
        } else {
            return float3(repeating: 0)
        }
    }

    var `struct`: PhysicsFieldStruct {
        return PhysicsFieldStruct(
            position: position,
            halfExtent: halfExtent ?? float3(repeating: -1))
    }
}
