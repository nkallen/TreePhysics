import Foundation
import XCTest
@testable import TreePhysics
import simd

class AutoTreeTests: XCTestCase {
    var config: AutoTreeConfig!
    var autoTree: AutoTree!
    var root: AutoTree.Node!
    var terminalBud: AutoTree.TerminalBud!

    override func setUp() {
        super.setUp()
        self.config = AutoTreeConfig()
        self.autoTree = AutoTree(config: config)
        self.root = autoTree.root()
        self.terminalBud = autoTree.terminalBud(position: .zero, orientation: .identity)
        root.addChild(terminalBud)
    }

    func testGrow() {
        // start with root -> terminalBud
        // transition to root -> internode -> terminalBud
        var (internode, buds) = terminalBud.grow(towards: [float3(1,1,0), float3(1,-1,0)])

        XCTAssertEqual(internode.position, terminalBud.position)
        XCTAssertEqual(simd_quatf(angle: -.pi/2, axis: .z), internode.orientation, accuracy: 0.0001)
        XCTAssertEqual(1, buds.count)

        var terminalBud = buds.first! as! AutoTree.TerminalBud
        XCTAssertEqual(.x * config.length, terminalBud.position, accuracy: 0.0001)
        XCTAssertEqual(simd_quatf(angle: config.phyllotacticAngle, axis: internode.orientation.heading) * internode.orientation, terminalBud.orientation)

        // transition to root -> internode -> [lateralBud] internode -> terminalBud
        (internode, buds) = terminalBud.grow(towards: [float3(10,0,0)])
        XCTAssertEqual(2, buds.count)
        terminalBud = buds.first(where: { $0 is AutoTree.TerminalBud }) as! AutoTree.TerminalBud
        let lateralBud = buds.first(where: { $0 is AutoTree.LateralBud }) as! AutoTree.LateralBud

        XCTAssertEqual(.x * config.length * 2, terminalBud.position, accuracy: 0.0001)
        XCTAssertEqual(
            (simd_quatf(angle: config.phyllotacticAngle, axis: internode.orientation.heading) * internode.orientation).normalized,
            terminalBud.orientation)

        XCTAssertEqual(internode.position, lateralBud.position)
        XCTAssertEqual(
            (simd_quatf(angle: config.branchingAngle, axis: internode.orientation.up) * internode.orientation).normalized,
            lateralBud.orientation)
    }
}
