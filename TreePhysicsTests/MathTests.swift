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

    func testEigenvaluesCardano() {
        XCTAssertEqual(
            float3(2 + sqrt(2), 2 - sqrt(2.0), 2),
            float3x3(columns: (
                float3(2,1,0),
                float3(1,2,1),
                float3(0,1,2)
            )).eigenvalues_cardano, accuracy: 0.0001)
    }
}
