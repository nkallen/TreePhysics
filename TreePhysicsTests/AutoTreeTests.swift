import Foundation
import XCTest
@testable import TreePhysics
import simd

class AutoTreeTests: XCTestCase {
    override func setUp() {
        super.setUp()
    }

    func testGrowOrientsInAverageDirection() {
        let bud = AutoTree.Bud(parent: nil, position: .zero, orientation: .identity)
        let (internode, buds) = bud.grow(towards: [float3(1,1,0), float3(1,-1,0)])
        XCTAssertEqual(internode.position, internode.position)
        XCTAssertEqual(simd_quatf(angle: -.pi/2, axis: .z).normalized, internode.orientation)
    }

    func testFoo() {
        // when a bud grows, it inserts a node into its parent, replacing itself
    }
}
