import Foundation
import Darwin
import simd

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
        if b < 0 {
            fatalError("The spring coefficient should never be 0") // FIXME remove
        } else {
            let r2 = (-b - sqrt(b2_4ac)) / (2.0*a)
            let r1 = c / (a * r2)
            return .realDistinct(r1, r2)
        }
    } else {
        let imaginaryPart = sqrt(-b2_4ac) / _2a
        let realPart = -b / _2a
        return .complex(realPart, imaginaryPart)
    }
}

func solve_differential(a: Float, b: Float, c: Float, g: Float, y_0: Float, y_ddt_0: Float) -> DifferentialSolution {
    switch solve_quadratic(a: a, b: b, c: c) {
    case let .complex(real, imaginary):
        let c1 = y_0
        let c2 = (y_ddt_0 - real * c1) / imaginary
        return .complex(c1: c1, c2: c2, λ: real, μ: imaginary, k: g/c)
    case let .real(r):
        let system = float2x2(columns: (float2(1, r), float2(0, 1)))
        let solution = system.inverse * float2(y_0, y_ddt_0)
        return .real(c1: solution.x, c2: solution.y, r: r, k: g/c)
    case let .realDistinct(r1, r2):
        let system = float2x2(columns: (float2(1, r1), float2(1, r2)))
        let solution = system.inverse * float2(y_0, y_ddt_0)
        return .realDistinct(c1: solution.x, c2: solution.y, r1: r1, r2: r2, k: g/c)
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

func matrix3x3_rotation(radians: Float) -> float3x3 {
    let cs = cosf(radians)
    let sn = sinf(radians)
    return matrix_float3x3.init(columns:
        (float3(cs, sn, 0),
         float3(-sn, cs, 0),
         float3(0, 0, 1)))
}

func matrix4x4_rotation(from: float3, to: float3) -> float4x4 {
    let from = normalize(from), to = normalize(to)

    let axis = cross(from, to)
    if length(axis) == 0 { return matrix_identity_float4x4 }

    let unitAxis = normalize(axis)
    let ct = dot(from, to)
    let st = length(axis)
    let ci = 1 - ct
    let x = unitAxis.x, y = unitAxis.y, z = unitAxis.z
    return matrix_float4x4.init(columns:(float4(    ct + x * x * ci, y * x * ci + z * st, z * x * ci - y * st, 0),
                                         float4(x * y * ci - z * st,     ct + y * y * ci, z * y * ci + x * st, 0),
                                         float4(x * z * ci + y * st, y * z * ci - x * st,     ct + z * z * ci, 0),
                                         float4(                  0,                   0,                   0, 1)))
}

func matrix4x4_rotation(radians: Float, axis: float3) -> matrix_float4x4 {
    let unitAxis = normalize(axis)
    let ct = cosf(radians)
    let st = sinf(radians)
    let ci = 1 - ct
    let x = unitAxis.x, y = unitAxis.y, z = unitAxis.z
    return matrix_float4x4.init(columns:(float4(    ct + x * x * ci, y * x * ci + z * st, z * x * ci - y * st, 0),
                                         float4(x * y * ci - z * st,     ct + y * y * ci, z * y * ci + x * st, 0),
                                         float4(x * z * ci + y * st, y * z * ci - x * st,     ct + z * z * ci, 0),
                                         float4(                  0,                   0,                   0, 1)))
}

func matrix4x4_rotation(rotation: float3) -> matrix_float4x4 {
    let pitch = rotation.x
    let yaw = rotation.y
    let roll = rotation.z

    let cb = cosf(pitch)
    let sb = sinf(pitch)
    let ch = cosf(yaw)
    let sh = sinf(yaw)
    let ca = cosf(roll)
    let sa = sinf(roll)

    return float4x4(columns:(
        float4(ch*ca,          sa,     -sh*ca,            0),
        float4(sh*sb-ch*sa*cb, ca*cb,  sh*sa*cb+ch*sb,    0),
        float4(ch*sa*sb+sh*cb, -ca*sb, -sh*sa*sb + ch*cb, 0),
        float4(0,              0,      0,                 1)))
}

func matrix3x3_rotation(rotation: float3) -> matrix_float3x3 {
    let pitch = rotation.x
    let yaw = rotation.y
    let roll = rotation.z

    let cb = cosf(pitch)
    let sb = sinf(pitch)
    let ch = cosf(yaw)
    let sh = sinf(yaw)
    let ca = cosf(roll)
    let sa = sinf(roll)

    return float3x3(columns:(
        float3(ch*ca,          sa,     -sh*ca),
        float3(sh*sb-ch*sa*cb, ca*cb,  sh*sa*cb+ch*sb),
        float3(ch*sa*sb+sh*cb, -ca*sb, -sh*sa*sb + ch*cb)))
}

func matrix3x3_rotation(rotation: matrix_float4x4) -> matrix_float3x3 {
    return float3x3(columns: (
        rotation[0].xyz,
        rotation[1].xyz,
        rotation[2].xyz))
}

func matrix4x4_translation(_ translationX: Float, _ translationY: Float, _ translationZ: Float) -> matrix_float4x4 {
    return matrix_float4x4.init(columns:(vector_float4(1, 0, 0, 0),
                                         vector_float4(0, 1, 0, 0),
                                         vector_float4(0, 0, 1, 0),
                                         vector_float4(translationX, translationY, translationZ, 1)))
}

func matrix4x4_scale(_ sx: Float, _ sy: Float, _ sz: Float) -> matrix_float4x4 {
    return matrix_float4x4.init(columns:(vector_float4(sx, 0, 0, 0),
                                         vector_float4(0, sy, 0, 0),
                                         vector_float4(0, 0, sz, 0),
                                         vector_float4(0, 0, 0, 1)))
}

func rotate(_ x: float3, by radians: Float, axis: float3) -> float3 {
    return (matrix4x4_rotation(radians: radians, axis: axis) * float4(x, 0)).xyz
}

func matrix3x3_rotation(from: matrix_float4x4, to: matrix_float4x4) -> matrix_float3x3 {
    return matrix_float3x3.init(columns:
        (float3(dot(to.columns.0, from.columns.0), dot(to.columns.1, from.columns.0), dot(to.columns.2, from.columns.0)),
         float3(dot(to.columns.0, from.columns.1), dot(to.columns.1, from.columns.1), dot(to.columns.2, from.columns.1)),
         float3(dot(to.columns.0, from.columns.2), dot(to.columns.1, from.columns.2), dot(to.columns.2, from.columns.2))))
}

func matrix3x3_rotation(from transform: matrix_float4x4) -> matrix_float3x3 {
    return float3x3(columns: (
        transform[0].xyz, transform[1].xyz, transform[2].xyz
    ))
}

func matrix3x3_translation(_ translationX: Float, _ translationY: Float) -> float3x3 {
    return matrix_float3x3.init(columns:(vector_float3(1, 0, 0),
                                         vector_float3(0, 1, 0),
                                         vector_float3(translationX, translationY, 1)))
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

    var crossMatrix: float3x3 {
        return matrix_float3x3.init(columns:
            (float3(0, self.z, -self.y),
             float3(-self.z, 0, self.x),
             float3(self.y, -self.x, 0)))
    }

    var allPositive: Bool {
        return x >= 0 && y >= 0 && z >= 0
    }
}

extension float3: Comparable {
    public static func < (lhs: float3, rhs: float3) -> Bool {
        return lhs.x < rhs.x && lhs.y < rhs.y && lhs.z < rhs.z
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
func sqr(_ x: matrix_float3x3) -> matrix_float3x3 {
    return matrix_multiply(x, x)
}

extension Float {
    static let e: Float = Float(Darwin.M_E)
}

extension matrix_float3x3 {
    init(_ double3x3: double3x3) {
        self = matrix_float3x3(columns: (
            float3(double3x3[0]),
            float3(double3x3[1]),
            float3(double3x3[2])
        ))
    }

    // unrolled: cf https://hal.archives-ouvertes.fr/hal-01550129/document
    var cholesky: float3x3 {
        var result = matrix_float3x3(0)

        // Load A into registers

        let a00 = self[0, 0]
        let a01 = self[0, 1], a11 = self[1, 1]
        let a02 = self[0, 2], a12 = self[1, 2], a22 = self[2, 2]

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

    var isValid: Bool {
        for i in 0..<3 {
            for j in 0..<3 {
                if self[i, j].isInfinite || self[i, j].isNaN {
                    return false
                }
            }
        }
        return true
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

extension float3 {
    init(_ h: half3) {
        self = float3(float(h.x), float(h.y), float(h.z))
    }
}

extension float3x3 {
    init(_ h: half3x3) {
        self = float3x3(float3(h[0]), float3(h[1]), float3(h[2]))
    }
}
