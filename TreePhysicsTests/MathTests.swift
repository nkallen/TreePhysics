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
            float3x3(columns: (
                float3(1.0/2, sqrt(2.0) / 2, 1.0/2),
                float3(1.0 / 2, -sqrt(2.0) / 2, 1.0 / 2),
                float3(1 / sqrt(2.0), 0, -1/sqrt(2.0))
            )),
            eigenvectors, accuracy: 0.0001)
    }

    func testTridiagonal() {
        let matrix = double3x3(columns: (
            double3(2,1,0),
            double3(1,2,1),
            double3(0,1,2)))

        let (Q, d, e) = matrix.tridiagonal
        let Q_transpose = Q.transpose
        let X = double3x3(columns: (
            double3(d[0],e[0],0),
            double3(e[0],d[1],e[1]),
            double3(0,e[1],d[2])))
        let Y = Q * X * Q_transpose

        XCTAssertEqual(matrix, Y)
    }

    func testEigenQL() {
        let matrix = double3x3(columns: (
            double3(2,1,0),
            double3(1,2,1),
            double3(0,1,2)))

        guard let (eigenvalues, eigenvectors) = matrix.eigen_ql else {
            XCTFail()
            return
        }

        XCTAssertEqual(
            double3(2 - sqrt(2), 2, 2 + sqrt(2.0)),
            eigenvalues, accuracy: 0.0001)

        XCTAssertEqual(
            double3x3(columns: (
                -double3(1.0 / 2, -sqrt(2.0) / 2, 1.0 / 2),
                double3(1 / sqrt(2.0), 0, -1/sqrt(2.0)),
                double3(1.0/2, sqrt(2.0) / 2, 1.0/2)
            )),
            eigenvectors, accuracy: 0.001)
    }

    func testRotation() {
        print(            matrix4x4_rotation(radians: .pi/4, axis: .z))
        print(matrix4x4_rotation(rotation: float3(0, 0, .pi/4)))
        XCTAssertEqual(
            matrix4x4_rotation(radians: .pi/4, axis: .z),
            matrix4x4_rotation(rotation: float3(0, 0, .pi/4)),
            accuracy: 0.0001)
    }
}
