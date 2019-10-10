import Foundation
import Darwin
import simd
import ShaderTypes

enum QuadraticSolution: Equatable {
    case real(Float)
    case realDistinct(Float, Float)
    case complex(Float, Float)
}

enum DifferentialSolution: Equatable {
    case real(c1: Float, c2: Float, r: Float, k: Float)
    case realDistinct(c1: Float, c2: Float, r1: Float, r2: Float, k: Float)
    case complex(c1: Float, c2: Float, λ: Float, μ: Float, k: Float)
}

func solve_quadratic(a: Float, b: Float, c: Float) -> QuadraticSolution {
    //    (-b +/- sqrt(b^2 - 4ac)) / 2a
    //    where r2 = c/ar1, cf: https://math.stackexchange.com/questions/311382/solving-a-quadratic-equation-with-precision-when-using-floating-point-variables
    let b2_4ac = b*b - 4.0*a*c
    let _2a = 2.0*a

    if b2_4ac == 0 {
        return .real(-b / _2a)
    } else if b2_4ac > 0 {
        let r2 = (-b - sqrt(b2_4ac)) / (2.0*a)
        let r1 = c / (a * r2)
        return .realDistinct(r1, r2)
    } else {
        let imaginaryPart = sqrt(-b2_4ac) / _2a
        let realPart = -b / _2a
        return .complex(realPart, imaginaryPart)
    }
}

func solve_differential(a: Float, b: Float, c: Float, g: Float, y_0: Float, y_ddt_0: Float) -> DifferentialSolution {
    let k = g/c
    let y_0_k = y_0 - k
    switch solve_quadratic(a: a, b: b, c: c) {
    case let .complex(real, imaginary):
        let c1 = y_0_k
        let c2 = (y_ddt_0 - real * c1) / imaginary
        return .complex(c1: c1, c2: c2, λ: real, μ: imaginary, k: k)
    case let .real(r):
        let system = float2x2(columns: (SIMD2<Float>(1, r), SIMD2<Float>(0, 1)))
        let solution = system.inverse * SIMD2<Float>(y_0_k, y_ddt_0)
        return .real(c1: solution.x, c2: solution.y, r: r, k: k)
    case let .realDistinct(r1, r2):
        let system = float2x2(columns: (SIMD2<Float>(1, r1), SIMD2<Float>(1, r2)))
        let solution = system.inverse * SIMD2<Float>(y_0_k, y_ddt_0)
        return .realDistinct(c1: solution.x, c2: solution.y, r1: r1, r2: r2, k: k)
    }
}

// Evaluate 2nd-order differential equation given its analytic solution
func evaluate(differential: DifferentialSolution, at t: Float) -> SIMD3<Float> {
    switch differential {
    case let .complex(c1: c1, c2: c2, λ: λ, μ: μ, k: k):
        let y = c1*powf(.e,λ*t)*cos(μ*t) + c2*powf(.e,λ*t)*sin(μ*t) + k
        let y_ddt = λ*c1*powf(.e,λ*t)*cos(μ*t) - μ*c1*powf(.e,λ*t)*sin(μ*t) +
            λ*c2*powf(.e,λ*t)*sin(μ*t) + μ*c2*powf(.e,λ*t)*cos(μ*t)
        let y_d2dt = λ*λ*c1*powf(.e,λ*t)*cos(μ*t) - μ*λ*c1*powf(.e,λ*t)*sin(μ*t) -
            (λ*μ*c1*powf(.e,λ*t)*sin(μ*t) + μ*μ*c1*powf(.e,λ*t)*cos(μ*t)) +
            λ*λ*c2*powf(.e,λ*t)*sin(μ*t) + μ*λ*c2*powf(.e,λ*t)*cos(μ*t) +
            λ*μ*c2*powf(.e,λ*t)*cos(μ*t) - μ*μ*c2*powf(.e,λ*t)*sin(μ*t)
        return SIMD3<Float>(y, y_ddt, y_d2dt)
    case let .real(c1: c1, c2: c2, r: r, k: k):
        let y = c1*powf(.e,r*t) + c2*t*powf(.e,r*t) + k
        let y_ddt = r*c1*powf(.e,r*t) +
            c2*powf(.e,r*t) + r*c2*t*powf(.e,r*t)
        let y_d2dt = r*r*c1*powf(.e,r*t) +
            r*c2*powf(.e,r*t) +
            r*c2*powf(.e,r*t) + r*r*c2*t*powf(.e,r*t)
        return SIMD3<Float>(y, y_ddt, y_d2dt)
    case let .realDistinct(c1: c1, c2: c2, r1: r1, r2: r2, k: k):
        let y = c1*powf(.e,r1*t) + c2*powf(.e,r2*t) + k
        let y_ddt = r1*c1*powf(.e,r1*t) + r2*c2*powf(.e,r2*t)
        let y_d2dt = r1*r1*c1 * powf(.e,r1*t) + r2*r2*c2 * powf(.e,r2*t)
        return SIMD3<Float>(y, y_ddt, y_d2dt)
    }
}

func evaluateDifferential(a: Float, b: Float, c: Float, g: Float, y_0: Float, y_ddt_0: Float, at t: Float) -> SIMD3<Float> {
    let solution = solve_differential(a: a, b: b, c: c, g: g, y_0: y_0, y_ddt_0: y_ddt_0)
    return evaluate(differential: solution, at: t)
}

extension Array where Element == Float {
    var sum: Float {
        return reduce(0, +)
    }
}

extension Array where Element == SIMD2<Float> {
    var sum: SIMD2<Float> {
        return reduce(.zero, +)
    }
}

extension Array where Element == SIMD3<Float> {
    var sum: SIMD3<Float> {
        return reduce(.zero, +)
    }
}

extension SIMD2 where Scalar == Float {
    init(_ d: SIMD2<Double>) {
        self = SIMD2<Float>(Float(d.x), Float(d.y))
    }

}

extension SIMD3 where Scalar == Float {
    static let x = SIMD3<Float>(1,0,0)
    static let y = SIMD3<Float>(0,1,0)
    static let z = SIMD3<Float>(0,0,1)

    init(_ double3: SIMD3<Double>) {
        self = SIMD3<Float>(Float(double3.x), Float(double3.y), Float(double3.z))
    }

    init(_ float2: SIMD2<Float>, _ z: Float) {
        self = SIMD3<Float>(float2.x, float2.y, z)
    }

    var xy: SIMD2<Float> {
        return SIMD2<Float>(x, y)
    }

    var xz: SIMD2<Float> {
        return SIMD2<Float>(x, z)
    }

    var skew: float3x3 {
        return float3x3(columns:
            (SIMD3<Float>(0, self.z, -self.y),
             SIMD3<Float>(-self.z, 0, self.x),
             SIMD3<Float>(self.y, -self.x, 0)))
    }

    var allPositive: Bool {
        return x >= 0 && y >= 0 && z >= 0
    }

    func `in`(min: SIMD3<Float>, max: SIMD3<Float>) -> Bool {
        return self.x >= min.x && self.x <= max.x &&
            self.y >= min.y && self.y <= max.y &&
            self.z >= min.z && self.z <= max.z
    }

    var isFinite: Bool {
        return x.isFinite && y.isFinite && z.isFinite
    }
}

extension SIMD4 where Scalar == Float {
    init(_ float3: SIMD3<Float>, _ w: Float) {
        self = SIMD4<Float>(float3.x, float3.y, float3.z, w)
    }

    var xyz: SIMD3<Float> {
        return SIMD3<Float>(x, y, z)
    }
}

extension SIMD3 where Scalar == Double {
    init(_ float3: SIMD3<Float>) {
        self = SIMD3<Double>(Double(float3.x), Double(float3.y), Double(float3.z))
    }
}

extension double3x3 {
    init(_ float3x3: float3x3) {
        self = double3x3(columns: (
            SIMD3<Double>(float3x3[0]),
            SIMD3<Double>(float3x3[1]),
            SIMD3<Double>(float3x3[2])
        ))
    }
}

@inline(__always)
func sqr(_ x: Float) -> Float {
    return x * x
}

@inline(__always)
func sqr(_ x: Double) -> Double {
    return x * x
}

@inline(__always)
func sqr(_ x: float3x3) -> float3x3 {
    return matrix_multiply(x, x)
}

extension Float {
    static let e: Float = Float(Darwin.M_E)
}

extension float3x3 {
    init(_ double3x3: double3x3) {
        self = float3x3(columns: (
            SIMD3<Float>(double3x3[0]),
            SIMD3<Float>(double3x3[1]),
            SIMD3<Float>(double3x3[2])
        ))
    }

    // unrolled: cf https://hal.archives-ouvertes.fr/hal-01550129/document
    var cholesky: float3x3 {
        var result = float3x3(0)

        // Load A into registers
        let  a0 = self[0],     a1 = self[1]
        let a00 = a0[0]
        let a01 = a0[1], a11 = a1[1]
        let a02 = a0[2], a12 = a1[2], a22 = self[2, 2]

        // Factorize A
        let sqrt_a00 = sqrt(a00)
        let l0 = SIMD3<Float>(a00, a01, a02) / sqrt_a00
        let l01 = l0.y, l02 = l0.z

        let l11 = sqrt(a11 - sqr(l01))
        let l12 = (a12 - l02 * l01) / l11

        let l22 = sqrt(a22 - sqr(l02) - sqr(l12))

        result[0] = l0
        result[1] = SIMD3<Float>(0, l11, l12)
        result[2, 2] = l22

        return result
    }

    var isSymmetric: Bool {
        return self == self.transpose
    }

    var isFinite: Bool {
        let (i, j, k) = columns
        return i.isFinite && j.isFinite && k.isFinite
    }

    var isPositiveDefinite: Bool {
        guard let (eigenvalues, _) = self.eigen_ql else { return false }
        return eigenvalues.allPositive
    }

    @inline(__always)
    func row(_ i: Int) -> SIMD3<Float> {
        return SIMD3<Float>(self[0, i], self[1, i], self[2, i])
    }
}

extension simd_quatf {
    public static let identity = simd_quatf(angle: 1, axis: .zero)

    var isFinite: Bool {
        return real.isFinite && imag.isFinite
    }

    var left: SIMD3<Float> {
        return act(.x)
    }

    var up: SIMD3<Float> {
        return act(.z)
    }

    var heading: SIMD3<Float> {
        return act(.y)
    }
}

func simd_bezier(_ v0: SIMD3<Float>, _ v1: SIMD3<Float>, _ v2: SIMD3<Float>, _ v3: SIMD3<Float>, t: Float) -> SIMD3<Float> {
    let v01 = mix(v0, v1, t: t)
    let v12 = mix(v1, v2, t: t)
    let v23 = mix(v2, v3, t: t)
    let v012 = mix(v01, v12, t: t)
    let v123 = mix(v12, v23, t: t)
    return mix(v012, v123, t: t)
}

func bezier(_ x0: Float, _ x1: Float, _ x2: Float, _ x3: Float, t: Float) -> Float {
    let x01 = mix(x0, x1, t: t)
    let x12 = mix(x1, x2, t: t)
    let x23 = mix(x2, x3, t: t)
    let x012 = mix(x01, x12, t: t)
    let x123 = mix(x12, x23, t: t)
    return mix(x012, x123, t: t)
}


func mix(_ x: Float, _ y: Float, t: Float) -> Float {
    return x * (1-t) + y * t
}
