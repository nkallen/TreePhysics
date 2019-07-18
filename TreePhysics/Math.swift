import Foundation
import Darwin
import simd

enum QuadraticSolution: Equatable {
    case real(Float)
    case realDistinct(Float, Float)
    case complex(Float, Float)
}

enum DifferentialSolution: Equatable {
    case real(c1: Float, c2: Float, r: Float, k: Float)
    case realDistinct(c1: Float, c2: Float, r1: Float, r2: Float, k: Float)
    case complex(c1: Float, c2: Float, lambda: Float, mu: Float, k: Float)
}

func solve_quadratic(a: Float, b: Float, c: Float) -> QuadraticSolution {
    //    (-b +/- sqrt(b^2 - 4ac)) / 2a
    let b2_4ac = b*b - 4*a*c
    let _2a = 1.0 / (2*a)
    if b2_4ac == 0 {
        let b_2a = -b * _2a
        return .real(b_2a)
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

func solve_differential(a: Float, b: Float, c: Float, g: Float, y_0: Float, y_ddt_0: Float) -> DifferentialSolution {
    switch solve_quadratic(a: a, b: b, c: c) {
    case let .complex(real, imaginary):
        let c1 = y_0
        let c2 = (y_ddt_0 - real * c1) / imaginary
        return .complex(c1: c1, c2: c2, lambda: real, mu: imaginary, k: g/c)
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
    case let .complex(c1: c1, c2: c2, lambda: lambda, mu: mu, k: k):
        let y = c1*powf(.e,lambda*t)*cos(mu*t) + c2*powf(.e,lambda*t)*sin(mu*t) + k
        let y_ddt = lambda*c1*powf(.e,lambda*t)*cos(mu*t) - mu*c1*powf(.e,lambda*t)*sin(mu*t) +
            lambda*c2*powf(.e,lambda*t)*sin(mu*t) + mu*c2*powf(.e,lambda*t)*cos(mu*t)
        let y_d2dt = lambda*lambda*c1*powf(.e,lambda*t)*cos(mu*t) - mu*lambda*c1*powf(.e,lambda*t)*sin(mu*t) -
            (lambda*mu*c1*powf(.e,lambda*t)*sin(mu*t) + mu*mu*c1*powf(.e,lambda*t)*cos(mu*t)) +
            lambda*lambda*c2*powf(.e,lambda*t)*sin(mu*t) + mu*lambda*c2*powf(.e,lambda*t)*cos(mu*t) +
            lambda*mu*c2*powf(.e,lambda*t)*cos(mu*t) - mu*mu*c2*powf(.e,lambda*t)*sin(mu*t)
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
        let y_ddt = r1*c1*powf(.e,r1*t) +
            r2*c2 * powf(.e,r2*t)
        let y_d2dt = r1*r1*c1 * powf(.e,r1*t) +
            r2*r2*c2 * powf(.e,r2*t)
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

func matrix3x3_translation(_ translationX: Float, _ translationY: Float) -> float3x3 {
    return matrix_float3x3.init(columns:(vector_float3(1, 0, 0),
                                         vector_float3(0, 1, 0),
                                         vector_float3(translationX, translationY, 1)))
}

extension float3 {
    init(_ float2: float2, _ z: Float) {
        self = float3(float2.x, float2.y, z)
    }

    var xy: float2 {
        return float2(x, y)
    }
}

func square(_ x: Float) -> Float {
    return x * x
}

extension Float {
    static let e: Float = Float(Darwin.M_E)
}
