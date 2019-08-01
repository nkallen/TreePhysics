import Foundation
import simd

protocol PhysicsField {
    var position: float3 { get }
    var halfExtent: float3? { get }
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
}
