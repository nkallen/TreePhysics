import Foundation
import XCTest
@testable import TreePhysics
import simd

class CylinderTests: XCTestCase {
    var cylinder: Cylinder!
    var arcLength: Float!

    override func setUp() {
        super.setUp()
        self.cylinder = Cylinder(radius: 1.0, height: 1.0, radialSegmentCount: 3)
        self.arcLength = 2.0 * .pi / 3
    }

    func testVertices() {
        XCTAssertEqual([
            float3(cos(0), -0.5, sin(0)),
            float3(cos(0), 0.5, sin(0)),

            float3(cos(arcLength), -0.5, sin(arcLength)),
            float3(cos(arcLength), 0.5, sin(arcLength)),

            float3(cos(2*arcLength), -0.5, sin(2*arcLength)),
            float3(cos(2*arcLength), 0.5, sin(2*arcLength)),
            ], cylinder.vertices)
    }

    func testIndices() {
        XCTAssertEqual([
            0,1,2,
            3,
            4,
            5,
            0
            ], cylinder.indices)
    }
}
