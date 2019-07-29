import XCTest
@testable import TreePhysics
import simd
import Accelerate

class MathTests: XCTestCase {
    func testCholesky() {
        XCTAssertEqual(
            float3x3(columns: (
                float3(2, 6, -8),
                float3(0, 1, 5),
                float3(0, 0, 3)
            )),
            float3x3(columns: (
                float3(4,12,-16),
                float3(12,37,-43),
                float3(-16,-43,98)
            )).cholesky)
    }

    func testEigenvaluesAnalytical() {
        let matrix = float3x3(columns: (
            float3(2,1,0),
            float3(1,2,1),
            float3(0,1,2)))
        guard let (eigenvalues, eigenvectors) = matrix.eigen_analytical else {
            XCTFail()
            return
        }

        XCTAssertEqual(
            float3(2 + sqrt(2), 2 - sqrt(2.0), 2),
            eigenvalues, accuracy: 0.0001)

        XCTAssertEqual(
            (matrix - matrix.eigenvalues_analytical.x * matrix_identity_float3x3).determinant,
            0, accuracy: 0.0001)

        XCTAssertEqual(
            float3x3(columns: (
                float3(1.0/2, sqrt(2.0) / 2, 1.0/2),
                float3(1.0 / 2, -sqrt(2.0) / 2, 1.0 / 2),
                float3(1 / sqrt(2.0), 0, -1/sqrt(2.0))
            )),
            eigenvectors, accuracy: 0.0001)
    }

    func testTridiagonal() {
        let matrix = float3x3(columns: (
            float3(2,1,0),
            float3(1,2,1),
            float3(0,1,2)))

        let (Q, d, e) = matrix.tridiagonal
        let Q_transpose = Q.transpose
        let X = float3x3(columns: (
            float3(d[0],e[0],0),
            float3(e[0],d[1],e[1]),
            float3(0,e[1],d[2])))
        let Y = Q * X * Q_transpose

        XCTAssertEqual(matrix, Y)
    }

    func testEigenQL() {
        let matrix = float3x3(columns: (
            float3(2,1,0),
            float3(1,2,1),
            float3(0,1,2)))

        guard let (eigenvalues, eigenvectors) = matrix.eigen_ql else {
            XCTFail()
            return
        }

        XCTAssertEqual(
            float3(2 - sqrt(2), 2, 2 + sqrt(2.0)),
            eigenvalues, accuracy: 0.0001)

        XCTAssertEqual(
            float3x3(columns: (
                -float3(1.0 / 2, -sqrt(2.0) / 2, 1.0 / 2),
                float3(1 / sqrt(2.0), 0, -1/sqrt(2.0)),
                float3(1.0/2, sqrt(2.0) / 2, 1.0/2)
            )),
            eigenvectors, accuracy: 0.001)
    }

    func testFoo() {
        let inertiaTensor_jointSpace = matrix_float3x3(columns: (
            float3(1.838548e-07, 6.280492e-08, 0.000000e+00),
            float3(6.280492e-08, 2.147834e-08, 0.000000e+00),
            float3(0.000000e+00, 0.000000e+00, 2.053115e-07) ))
        let L = inertiaTensor_jointSpace.cholesky
        let L_inverse = L.inverse, L_transpose_inverse = L.transpose.inverse
        let k: Float = 27423.6777
        let A = L_inverse * (k * matrix_identity_float3x3) * L_transpose_inverse
//        let (Λ, X) = A.eigen_ql!

        print(A.tridiagonal)
    }
}
