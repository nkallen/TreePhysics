import Foundation
import XCTest
@testable import TreePhysics
import simd

class CylinderTests: XCTestCase {
    func testVertices() {
        let cylinder = Cylinder(radius: 1.0, height: 1.0, radialSegmentCount: 3)
        let arcLength: Float = 2.0 * .pi / 3
        XCTAssertEqual([
            float3(cos(0), -0.5, sin(0)),
            float3(cos(0), 0.5, sin(0)),

            float3(cos(arcLength), -0.5, sin(arcLength)),
            float3(cos(arcLength), 0.5, sin(arcLength)),

            float3(cos(2*arcLength), -0.5, sin(2*arcLength)),
            float3(cos(2*arcLength), 0.5, sin(2*arcLength)),
            ], cylinder.vertices)
    }
}
