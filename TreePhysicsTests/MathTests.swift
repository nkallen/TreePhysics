import XCTest
@testable import TreePhysics
import simd

class MathTests: XCTestCase {
    func testCholesky() {
        XCTAssertEqual(
            float3x3(columns: (
                SIMD3<Float>(2, 6, -8),
                SIMD3<Float>(0, 1, 5),
                SIMD3<Float>(0, 0, 3)
            )),
            float3x3(columns: (
                SIMD3<Float>(4,12,-16),
                SIMD3<Float>(12,37,-43),
                SIMD3<Float>(-16,-43,98)
            )).cholesky)
    }
    
    func testEigenvaluesAnalytical() {
        let matrix = float3x3(columns: (
            SIMD3<Float>(2,1,0),
            SIMD3<Float>(1,2,1),
            SIMD3<Float>(0,1,2)))
        guard let (eigenvalues, eigenvectors) = matrix.eigen_analytical else {
            XCTFail()
            return
        }

        XCTAssertEqual(
            SIMD3<Float>(2 + sqrt(2), 2 - sqrt(2.0), 2),
            eigenvalues, accuracy: 0.0001)

        XCTAssertEqual(
            float3x3(columns: (
                SIMD3<Float>(1.0/2, sqrt(2.0) / 2, 1.0/2),
                SIMD3<Float>(1.0 / 2, -sqrt(2.0) / 2, 1.0 / 2),
                SIMD3<Float>(1 / sqrt(2.0), 0, -1/sqrt(2.0))
            )),
            eigenvectors, accuracy: 0.0001)
    }

    func testTridiagonal() {
        let matrix = double3x3(columns: (
            SIMD3<Double>(2,1,0),
            SIMD3<Double>(1,2,1),
            SIMD3<Double>(0,1,2)))

        let (Q, d, e) = matrix.tridiagonal
        let Q_transpose = Q.transpose
        let X = double3x3(columns: (
            SIMD3<Double>(d[0],e[0],0),
            SIMD3<Double>(e[0],d[1],e[1]),
            SIMD3<Double>(0,e[1],d[2])))
        let Y = Q * X * Q_transpose

        XCTAssertEqual(matrix, Y)
    }

    func testEigenQL_0() {
        let matrix = double3x3(columns: (
            SIMD3<Double>(2,1,0),
            SIMD3<Double>(1,2,1),
            SIMD3<Double>(0,1,2)))

        guard let (eigenvalues, eigenvectors) = matrix.eigen_ql else {
            XCTFail()
            return
        }

        XCTAssertEqual(
            SIMD3<Double>(2 - sqrt(2), 2, 2 + sqrt(2.0)),
            eigenvalues, accuracy: 0.0001)

        XCTAssertEqual(
            double3x3(columns: (
                -SIMD3<Double>(1.0 / 2, -sqrt(2.0) / 2, 1.0 / 2),
                SIMD3<Double>(1 / sqrt(2.0), 0, -1/sqrt(2.0)),
                SIMD3<Double>(1.0/2, sqrt(2.0) / 2, 1.0/2)
            )),
            eigenvectors, accuracy: 0.001)
    }

    func testEigenQL_1() {
        let matrix = float3x3(columns: (
            SIMD3<Float>(1.0/3,-1.9254154e-10,2.9612822e-08),
            SIMD3<Float>(-1.9254154e-10,0.48408476,-0.048982114),
            SIMD3<Float>(2.9612824e-08,-0.04898209,0.34924862)
        ))

        let (Q, d, e) = double3x3(matrix).tridiagonal
        XCTAssertEqual(double3x3(columns: (
            SIMD3<Double>(1,0,0),
            SIMD3<Double>(0,-0.006502,0.999979),
            SIMD3<Double>(0,0.999979,0.006502)
        )), Q, accuracy: 0.001)
        XCTAssertEqual(SIMD3<Double>(1/3,0.349891,0.483442), d, accuracy: 0.0001)
        XCTAssertEqual(SIMD3<Double>(0,-0.049855), e, accuracy: 0.0001)
        XCTAssertEqual(
            SIMD3<Float>(1/3,1/3,1/2),
            matrix.eigen_ql!.0, accuracy: 0.0001)
    }
}

class QuadraticTests: XCTestCase {
    func testRealDistinct() {
        XCTAssertEqual(solve_quadratic(a: 1, b: 11, c: 24),
                       .realDistinct(-3, -8))
    }

    func testReal() {
        XCTAssertEqual(solve_quadratic(a: 1, b: -4, c: 4),
                       .real(2))
    }

    func testComplex() {
        XCTAssertEqual(solve_quadratic(a: 1, b: -4, c: 9),
                       .complex(2, sqrt(5)))
    }
}

class DifferentialTests: XCTestCase {
    func testRealDistinct() {
        let actual = solve_differential(a: 1, b: 11, c: 24, g: 0, y_0: 0, y_ddt_0: -7)
        let expected = DifferentialSolution.realDistinct(c1: -7.0/5, c2: 7.0/5, r1: -3, r2: -8, k: 0)
        XCTAssertEqual(actual, expected, accuracy: 0.0001)
        XCTAssertEqual(evaluate(differential: actual, at: 0), SIMD3<Float>(0, -7, 77))
    }

    func testReal() {
        let actual = solve_differential(a: 1, b: -4, c: 4, g: 0, y_0: 12, y_ddt_0: -3)
        let expected = DifferentialSolution.real(c1: 12, c2: -27, r: 2, k: 0)
        XCTAssertEqual(actual, expected)
        XCTAssertEqual(evaluate(differential: actual, at: 0), SIMD3<Float>(12, -3, -60))
    }

    func testComplex() {
        let actual = solve_differential(a: 1, b: -4, c: 9, g: 0, y_0: 0, y_ddt_0: -8)
        let expected = DifferentialSolution.complex(c1: 0, c2: -8/sqrt(5), λ: 2, μ: sqrt(5), k: 0)
        XCTAssertEqual(actual, expected)
        XCTAssertEqual(evaluate(differential: actual, at: 0), SIMD3<Float>(0, -8, -32))
    }
}
