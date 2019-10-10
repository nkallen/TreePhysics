import Foundation
import simd

public class Noise {
    public init() {}

    public func random(_ x: Int) -> Float {
        return random(Float(x))
    }

    public func random(_ x: Float) -> Float {
        return modf(sin(x)*1.0).1
    }

    public func random(_ st: SIMD2<Float>) -> Float {
        return modf(sin(dot(st,
                            SIMD2<Float>(12.9898,78.233))) *
            43758.5453123).1
    }

    // Based on Morgan McGuire @morgan3d
    // https://www.shadertoy.com/view/4dS3Wd
    func noise(_ st: SIMD2<Float>) -> Float {
        let i = floor(st)
        let f = fract(st)

        // Four corners in 2D of a tile
        let a = random(i)
        let b = random(i + SIMD2<Float>(1.0, 0.0))
        let c = random(i + SIMD2<Float>(0.0, 1.0))
        let d = random(i + SIMD2<Float>(1.0, 1.0))

        var u: SIMD2<Float> = f * f
        u *= (3.0 - 2.0 * f)

        return mix(a, b, t: u.x) +
            (c - a) * u.y * (1.0 - u.x) +
            (d - b) * u.x * u.y;
    }

    let octaves = 4
    let lacunarity: Float = 1.5
    let gain: Float = 0.75
    public func fbm(_ st: SIMD2<Float>, amplitude: Float) -> Float {
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
