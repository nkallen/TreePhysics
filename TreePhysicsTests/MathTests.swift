import XCTest
@testable import TreePhysics
import simd

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

        XCTAssertEqual(
            float3(2 + sqrt(2), 2 - sqrt(2.0), 2),
            matrix.eigenvalues_analytical, accuracy: 0.0001)

        XCTAssertEqual(
            (matrix - matrix.eigenvalues_analytical.x * matrix_identity_float3x3).determinant,
            0, accuracy: 0.0001)

        XCTAssertEqual(
            float3x3(columns: (
                float3(1.0/2, sqrt(2.0) / 2, 1.0/2),
                float3(1.0 / 2, -sqrt(2.0) / 2, 1.0 / 2),
                float3(1 / sqrt(2.0), 0, -1/sqrt(2.0))
            )),
            matrix.eigenvectors_analytical, accuracy: 0.0001)
        print(matrix.eigenvectors_analytical)
    }

    func testEigenvectorsAnalytical() {
        let matrix = double3x3(columns: (
            double3(-2,-2,4),
            double3(-4,1,2),
            double3(2,2,5)
        ))
        XCTAssertEqual(
            double3(6, -5, 3),
            matrix.eigenvalues_analytical, accuracy: 0.0001)
        XCTAssertEqual(
            (matrix - matrix.eigenvalues_analytical.x * matrix_identity_double3x3).determinant,
            0)
        XCTAssertEqual(
            (matrix - matrix.eigenvalues_analytical.y * matrix_identity_double3x3).determinant,
            0)
        XCTAssertEqual(
            (matrix - matrix.eigenvalues_analytical.z * matrix_identity_double3x3).determinant,
            0)
    }

    func testEigenvectorsAnalyitcal() {
        let matrix = float3x3(columns: (
            float3(-2,-2,4),
            float3(-4,1,2),
            float3(2,2,5)
        ))
        XCTAssertEqual(
            float3(6, -5, 3),
            matrix.eigenvalues_analytical, accuracy: 0.0001)

        XCTAssertEqual(
            float3x3(columns: (
                normalize(float3(1, -3.0/2, -1.0/2)),
                normalize(float3(1, -1.0/2, 1.0/2)),
                normalize(float3(1, 6, 16))
            )),
            matrix.eigenvectors_analytical, accuracy: 0.0001)
        print(matrix.eigenvectors_analytical)
    }
}
