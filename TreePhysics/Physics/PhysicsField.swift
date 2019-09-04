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
        let delta = self.position - position
        let distance = length(delta)
        if (distance > 0) {
            let direction = normalize(delta)
            // NOTE: this is a bell-shaped curve, so objects very far and very near are weakly affected
            let force = direction * a * powf(.e, -sqr(distance - b)/(2*c))
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

final class WindField: PhysicsField {
    var position = float3.zero

    var halfExtent: float3? = nil

    var `struct`: PhysicsFieldStruct {
        return PhysicsFieldStruct(
            position: position,
            halfExtent: halfExtent ?? float3(repeating: -1))
    }

    func eval(position: float3, velocity: float3, mass: Float, time: TimeInterval) -> float3 {
        let t: Float = 0.1*(-Float(time))

        let magnitude = fbm(floor(position.xy*100) + t) * 0.01
        let direction = float3(-1,0,0)
        return magnitude * direction
    }

    func random(_ st: float2) -> Float {
        return modf(sin(dot(st,
                            float2(12.9898,78.233))) *
            43758.5453123).1
    }

    // Based on Morgan McGuire @morgan3d
    // https://www.shadertoy.com/view/4dS3Wd
    func noise(_ st: float2) -> Float {
        let i = floor(st)
        let f = fract(st)

        // Four corners in 2D of a tile
        let a = random(i)
        let b = random(i + float2(1.0, 0.0))
        let c = random(i + float2(0.0, 1.0))
        let d = random(i + float2(1.0, 1.0))

        let u = f * f * (3.0 - 2.0 * f);

        return mix(a, b, u.x) +
            (c - a) * u.y * (1.0 - u.x) +
                (d - b) * u.x * u.y;
    }

    let octaves = 6
    func fbm(_ st: float2) -> Float {
        // Initial values
        var st = st
        var value: Float = 0.0
        var amplitude: Float = 0.5

        // Loop of octaves
        for _ in 0..<octaves {
            value += amplitude * noise(st)
            st *= 2.0
            amplitude *= 0.5
        }
        return value
    }
}

func mix(_ x: Float, _ y: Float, _ t: Float) -> Float {
    return x * (1-t) + y * t
}
