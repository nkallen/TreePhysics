import Foundation
import Darwin
import simd
import ShaderTypes

typealias half3 = vector_half3

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
        let system = float2x2(columns: (float2(1, r), float2(0, 1)))
        let solution = system.inverse * float2(y_0_k, y_ddt_0)
        return .real(c1: solution.x, c2: solution.y, r: r, k: k)
    case let .realDistinct(r1, r2):
        let system = float2x2(columns: (float2(1, r1), float2(1, r2)))
        let solution = system.inverse * float2(y_0_k, y_ddt_0)
        return .realDistinct(c1: solution.x, c2: solution.y, r1: r1, r2: r2, k: k)
    }
}

// Evaluate 2nd-order differential equation given its analytic solution
func evaluate(differential: DifferentialSolution, at t: Float) -> float3 {
    switch differential {
    case let .complex(c1: c1, c2: c2, λ: λ, μ: μ, k: k):
        let y = c1*powf(.e,λ*t)*cos(μ*t) + c2*powf(.e,λ*t)*sin(μ*t) + k
        let y_ddt = λ*c1*powf(.e,λ*t)*cos(μ*t) - μ*c1*powf(.e,λ*t)*sin(μ*t) +
            λ*c2*powf(.e,λ*t)*sin(μ*t) + μ*c2*powf(.e,λ*t)*cos(μ*t)
        let y_d2dt = λ*λ*c1*powf(.e,λ*t)*cos(μ*t) - μ*λ*c1*powf(.e,λ*t)*sin(μ*t) -
            (λ*μ*c1*powf(.e,λ*t)*sin(μ*t) + μ*μ*c1*powf(.e,λ*t)*cos(μ*t)) +
            λ*λ*c2*powf(.e,λ*t)*sin(μ*t) + μ*λ*c2*powf(.e,λ*t)*cos(μ*t) +
            λ*μ*c2*powf(.e,λ*t)*cos(μ*t) - μ*μ*c2*powf(.e,λ*t)*sin(μ*t)
        return float3(y, y_ddt, y_d2dt)
    case let .real(c1: c1, c2: c2, r: r, k: k):
        let y = c1*powf(.e,r*t) + c2*t*powf(.e,r*t) + k
        let y_ddt = r*c1*powf(.e,r*t) +
            c2*powf(.e,r*t) + r*c2*t*powf(.e,r*t)
        let y_d2dt = r*r*c1*powf(.e,r*t) +
            r*c2*powf(.e,r*t) +
            r*c2*powf(.e,r*t) + r*r*c2*t*powf(.e,r*t)
        return float3(y, y_ddt, y_d2dt)
    case let .realDistinct(c1: c1, c2: c2, r1: r1, r2: r2, k: k):
        let y = c1*powf(.e,r1*t) + c2*powf(.e,r2*t) + k
        let y_ddt = r1*c1*powf(.e,r1*t) + r2*c2*powf(.e,r2*t)
        let y_d2dt = r1*r1*c1 * powf(.e,r1*t) + r2*r2*c2 * powf(.e,r2*t)
        return float3(y, y_ddt, y_d2dt)
    }
}

func evaluateDifferential(a: Float, b: Float, c: Float, g: Float, y_0: Float, y_ddt_0: Float, at t: Float) -> float3 {
    let solution = solve_differential(a: a, b: b, c: c, g: g, y_0: y_0, y_ddt_0: y_ddt_0)
    return evaluate(differential: solution, at: t)
}

extension Array where Element == Float {
    var sum: Float {
        return reduce(0, +)
    }
}

extension Array where Element == float2 {
    var sum: float2 {
        return reduce(float2.zero, +)
    }
}

extension Array where Element == float3 {
    var sum: float3 {
        return reduce(float3.zero, +)
    }
}

extension float2 {
    init(_ double2: double2) {
        self = float2(Float(double2.x), Float(double2.y))
    }

}

extension float3 {
    static let x = float3(1,0,0)
    static let y = float3(0,1,0)
    static let z = float3(0,0,1)

    init(_ double3: double3) {
        self = float3(Float(double3.x), Float(double3.y), Float(double3.z))
    }

    init(_ float2: float2, _ z: Float) {
        self = float3(float2.x, float2.y, z)
    }

    var xy: float2 {
        return float2(x, y)
    }

    var xz: float2 {
        return float2(x, z)
    }

    var skew: float3x3 {
        return float3x3(columns:
            (float3(0, self.z, -self.y),
             float3(-self.z, 0, self.x),
             float3(self.y, -self.x, 0)))
    }

    var allPositive: Bool {
        return x >= 0 && y >= 0 && z >= 0
    }

    func `in`(min: float3, max: float3) -> Bool {
        return self.x >= min.x && self.x <= max.x &&
            self.y >= min.y && self.y <= max.y &&
            self.z >= min.z && self.z <= max.z
    }

    var isFinite: Bool {
        return x.isFinite && y.isFinite && z.isFinite
    }
}

extension float4 {
    init(_ float3: float3, _ w: Float) {
        self = float4(float3.x, float3.y, float3.z, w)
    }

    var xyz: float3 {
        return float3(x, y, z)
    }
}

extension double3 {
    init(_ float3: float3) {
        self = double3(Double(float3.x), Double(float3.y), Double(float3.z))
    }
}

extension double3x3 {
    init(_ float3x3: float3x3) {
        self = double3x3(columns: (
            double3(float3x3[0]),
            double3(float3x3[1]),
            double3(float3x3[2])
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
            float3(double3x3[0]),
            float3(double3x3[1]),
            float3(double3x3[2])
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
        let l0 = float3(a00, a01, a02) / sqrt_a00
        let l01 = l0.y, l02 = l0.z

        let l11 = sqrt(a11 - sqr(l01))
        let l12 = (a12 - l02 * l01) / l11

        let l22 = sqrt(a22 - sqr(l02) - sqr(l12))

        result[0] = l0
        result[1] = float3(0, l11, l12)
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
    func row(_ i: Int) -> float3 {
        return float3(self[0, i], self[1, i], self[2, i])
    }
}

extension half3 {
    init(_ x: Float, _ y: Float, _ z: Float) {
        self.init()
        self.x = half(x)
        self.y = half(y)
        self.z = half(z)
    }

    init(_ x: half, _ y: half, _ z: half) {
        self.init()
        self.x = x
        self.y = y
        self.z = z
    }

    init(_ vec: float3) {
        self.init(half(vec.x), half(vec.y), half(vec.z))
    }
}

extension half3: Equatable {
    public static func == (lhs: vector_half3, rhs: vector_half3) -> Bool {
        return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z
    }
}

extension half3x3 {
    init(_ scalar: half) {
        self.init()
        self.columns = (
            half3(scalar, scalar, scalar),
            half3(scalar, scalar, scalar),
            half3(scalar, scalar, scalar)
        )
    }

    init(_ mat: float3x3) {
        self.init()
        self.columns = (
            half3(mat[0]),
            half3(mat[1]),
            half3(mat[2])
        )
    }

    subscript(_ i: Int) -> half3 {
        switch i {
        case 0:
            return columns.0
        case 1:
            return columns.1
        case 2:
            return columns.2
        default: fatalError()
        }
    }
}

func half(_ x: Float) -> half {
    return float16_from_float32(x)
}

func float(_ x: half) -> Float {
    return float32_from_float16(x)
}

extension half {
    init (_ x: Float) {
        self = float16_from_float32(x)
    }
}

extension simd_quatf {
    static let identity = simd_quatf(angle: 1, axis: float3.zero)

    var isFinite: Bool {
        return real.isFinite && imag.isFinite
    }

    var left: float3 {
        return act(.x)
    }

    var up: float3 {
        return act(.z)
    }

    var heading: float3 {
        return act(.y)
    }
}

func simd_bezier(_ v0: float3, _ v1: float3, _ v2: float3, _ v3: float3, t: Float) -> float3 {
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
