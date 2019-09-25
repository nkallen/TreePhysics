import Foundation
import simd

class Noise {
    func random(_ x: Int) -> Float {
        return random(Float(x))
    }

    func random(_ x: Float) -> Float {
        return modf(sin(x)*1.0).1
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

        var u: float2 = f * f
        u *= (3.0 - 2.0 * f)

        return mix(a, b, t: u.x) +
            (c - a) * u.y * (1.0 - u.x) +
            (d - b) * u.x * u.y;
    }

    let octaves = 4
    let lacunarity: Float = 1.5
    let gain: Float = 0.75
    func fbm(_ st: float2, amplitude: Float) -> Float {
        var st = st
        var amplitude = amplitude
        var value: Float = 0.0

        for _ in 0..<octaves {
            value += amplitude * noise(st)
            st *= lacunarity
            amplitude *= gain
        }
        return value
    }

}
