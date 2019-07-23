import Foundation
import XCTest
@testable import TreePhysics
import simd

class CylinderPenTests: XCTestCase {
    var pen: CylinderPen!
    var arcLength: Float!

    override func setUp() {
        super.setUp()
        self.pen = CylinderPen(radialSegmentCount: 3)
        self.arcLength = 2.0 * .pi / 3

        pen.start(at: float2.zero, thickness: 1)
        pen.cont(distance: 1, tangent: float2(0,1), thickness: .pi)
    }

    func testVertices() {
        XCTAssertEqual([
            float3(cos(0), 0, sin(0)),
            float3(cos(0), 1, sin(0)),

            float3(cos(arcLength), 0, sin(arcLength)),
            float3(cos(arcLength), 1, sin(arcLength)),

            float3(cos(2*arcLength), 0, sin(2*arcLength)),
            float3(cos(2*arcLength), 1, sin(2*arcLength)),
            ], pen.vertices)
    }

    func testIndices() {
        XCTAssertEqual([
            0,1,2,
            3,
            4,
            5,
            0,
            1
            ], pen.indices)
    }
}