import Foundation
import Darwin
import simd

enum QuadraticSolution: Equatable {
    case real(Double)
    case realDistinct(Double, Double)
    case complex(Double, Double)
}

enum DifferentialSolution: Equatable {
    case real(c1: Double, c2: Double, r: Double, k: Double)
    case realDistinct(c1: Double, c2: Double, r1: Double, r2: Double, k: Double)
    case complex(c1: Double, c2: Double, lambda: Double, mu: Double, k: Double)
}

func solve_quadratic(a: Double, b: Double, c: Double) -> QuadraticSolution {
    //    (-b +/- sqrt(b^2 - 4ac)) / 2a
    let b2_4ac = b*b - 4.0*a*c
    let _2a = 1.0 / (2.0*a)
    if b2_4ac == 0 {
        let b_2a = -b * _2a
        return .real(Double(b_2a))
    } else if b2_4ac > 0 {
        let b_2a = -b * _2a
        let sqrt_b2_4ac_2a = sqrt(b2_4ac) * _2a
        return .realDistinct(b_2a + sqrt_b2_4ac_2a, b_2a - sqrt_b2_4ac_2a)
    } else {
        let imaginaryPart = sqrt(-b2_4ac) * _2a
        let realPart = -b * _2a
        return .complex(realPart, imaginaryPart)
    }
}

func solve_differential(a: Double, b: Double, c: Double, g: Double, y_0: Double, y_ddt_0: Double) -> DifferentialSolution {
    switch solve_quadratic(a: a, b: b, c: c) {
    case let .complex(real, imaginary):
        let c1 = y_0
        let c2 = (y_ddt_0 - real * c1) / imaginary
        return .complex(c1: c1, c2: c2, lambda: real, mu: imaginary, k: g/c)
    case let .real(r):
        let system = double2x2(columns: (double2(1, r), double2(0, 1)))
        let solution = system.inverse * double2(y_0, y_ddt_0)
        return .real(c1: solution.x, c2: solution.y, r: r, k: g/c)
    case let .realDistinct(r1, r2):
        let system = double2x2(columns: (double2(1, r1), double2(1, r2)))
        let solution = system.inverse * double2(y_0, y_ddt_0)
        return .realDistinct(c1: solution.x, c2: solution.y, r1: r1, r2: r2, k: g/c)
    }
}

// Evaluate 2nd-order differential equation given its analytic solution
func evaluate(differential: DifferentialSolution, at t: Double) -> float3 {
    switch differential {
    case let .complex(c1: c1, c2: c2, lambda: lambda, mu: mu, k: k):
        let y = c1*pow(.e,lambda*t)*cos(mu*t) + c2*pow(.e,lambda*t)*sin(mu*t) + k
        let y_ddt = lambda*c1*pow(.e,lambda*t)*cos(mu*t) - mu*c1*pow(.e,lambda*t)*sin(mu*t) +
            lambda*c2*pow(.e,lambda*t)*sin(mu*t) + mu*c2*pow(.e,lambda*t)*cos(mu*t)
        let y_d2dt = lambda*lambda*c1*pow(.e,lambda*t)*cos(mu*t) - mu*lambda*c1*pow(.e,lambda*t)*sin(mu*t) -
            (lambda*mu*c1*pow(.e,lambda*t)*sin(mu*t) + mu*mu*c1*pow(.e,lambda*t)*cos(mu*t)) +
            lambda*lambda*c2*pow(.e,lambda*t)*sin(mu*t) + mu*lambda*c2*pow(.e,lambda*t)*cos(mu*t) +
            lambda*mu*c2*pow(.e,lambda*t)*cos(mu*t) - mu*mu*c2*pow(.e,lambda*t)*sin(mu*t)
        return float3(Float(y), Float(y_ddt), Float(y_d2dt))
    case let .real(c1: c1, c2: c2, r: r, k: k):
        let y = c1*pow(.e,r*t) + c2*t*pow(.e,r*t) + k
        let y_ddt = r*c1*pow(.e,r*t) +
            c2*pow(.e,r*t) + r*c2*t*pow(.e,r*t)
        let y_d2dt = r*r*c1*pow(.e,r*t) +
            r*c2*pow(.e,r*t) +
            r*c2*pow(.e,r*t) + r*r*c2*t*pow(.e,r*t)
        return float3(Float(y), Float(y_ddt), Float(y_d2dt))
    case let .realDistinct(c1: c1, c2: c2, r1: r1, r2: r2, k: k):
        let y = c1*pow(.e,r1*t) + c2*pow(.e,r2*t) + k
        let y_ddt = r1*c1*pow(.e,r1*t) +
            r2*c2 * pow(.e,r2*t)
        let y_d2dt = r1*r1*c1 * pow(.e,r1*t) +
            r2*r2*c2 * pow(.e,r2*t)
        return float3(Float(y), Float(y_ddt), Float(y_d2dt))
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

func matrix3x3_translation(_ translationX: Float, _ translationY: Float) -> float3x3 {
    return matrix_float3x3.init(columns:(vector_float3(1, 0, 0),
                                         vector_float3(0, 1, 0),
                                         vector_float3(translationX, translationY, 1)))
}

extension float3 {
    static let x = float3(1,0,0)
    static let y = float3(0,1,0)
    static let z = float3(0,0,1)

    init(_ float2: float2, _ z: Float) {
        self = float3(float2.x, float2.y, z)
    }

    var xy: float2 {
        return float2(x, y)
    }

    var cross_matrix: float3x3 {
        return matrix_float3x3.init(columns:
            (float3(0, self.z, -self.y),
             float3(-self.z, 0, self.x),
             float3(self.y, -self.x, 0)))
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

func square(_ x: Float) -> Float {
    return x * x
}

func square(_ x: matrix_float3x3) -> matrix_float3x3 {
    return matrix_multiply(x, x)
}

extension Double {
    static let e: Double = Darwin.M_E
}
