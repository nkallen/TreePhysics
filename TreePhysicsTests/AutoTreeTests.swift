import Foundation
import XCTest
@testable import TreePhysics
import simd

class AutoTreeTests: XCTestCase {
    override func setUp() {
        super.setUp()
    }

    func testGrowOrientsInAverageDirection() {
        let bud = AutoTree.Bud(position: .zero, orientation: .identity, producesLateralBud: true)
        let node = bud.grow(towards: [float3(1,1,0), float3(1,-1,0)])
        XCTAssertEqual(bud.position, node.position)
        XCTAssertEqual(simd_quatf(angle: -.pi/2, axis: .z).normalized, node.orientation)
    }

    func testGrowProducesBuds() {
        let bud = AutoTree.Bud(position: .zero, orientation: .identity, producesLateralBud: true)
        let node = bud.grow(towards: [float3(0,1,0)])
        XCTAssertNotNil(node.lateralBud)
        XCTAssertNotNil(node.terminalBud)
        XCTAssertNil(node.lateralBud!.grow(towards: [float3(0,2,0)]).lateralBud)
        XCTAssertNotNil(node.terminalBud!.grow(towards: [float3(0,2,0)]).lateralBud)
    }

    func testFoo() {
        // when a bud grows, it inserts a node into its parent, replacing itself
    }
}
