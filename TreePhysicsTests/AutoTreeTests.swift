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
        self.autoTree = AutoTree(config)
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
        XCTAssertEqual(.x * config.internodeLength, terminalBud.position, accuracy: 0.0001)
        XCTAssertEqual(simd_quatf(angle: config.phyllotacticAngle, axis: internode.orientation.heading) * internode.orientation, terminalBud.orientation)

        // transition to root -> internode -> [lateralBud] internode -> terminalBud
        (internode, buds) = terminalBud.grow(towards: [float3(10,0,0)])
        XCTAssertEqual(2, buds.count)
        terminalBud = buds.first(where: { $0 is AutoTree.TerminalBud }) as! AutoTree.TerminalBud
        let lateralBud = buds.first(where: { $0 is AutoTree.LateralBud }) as! AutoTree.LateralBud

        XCTAssertEqual(.x * config.internodeLength * 2, terminalBud.position, accuracy: 0.0001)
        XCTAssertEqual(
            (simd_quatf(angle: config.phyllotacticAngle, axis: internode.orientation.heading) * internode.orientation).normalized,
            terminalBud.orientation)

        XCTAssertEqual(internode.position, lateralBud.position)
        XCTAssertEqual(
            (simd_quatf(angle: config.branchingAngle, axis: internode.orientation.up) * internode.orientation).normalized,
            lateralBud.orientation)
    }

    func testTerminalBranchCount() {
        XCTAssertEqual(0, root.terminalBranchCount)
        let (_, buds) = terminalBud.grow(towards: [])
        XCTAssertEqual(1, root.terminalBranchCount)
        for bud in buds {
            let (_, buds) = bud.grow(towards: [])
            for bud in buds {
                _ = bud.grow(towards: [])
            }
        }
        XCTAssertEqual(2, root.terminalBranchCount)
    }

    func testShadowGrid() {
        let config = AutoTree.ShadowGridConfig(cellSize: 1)
        let grid = AutoTree.ShadowGrid(config)
        grid[float3(3,3,3)] += 1
        for i in 0..<8 {
            for k in 0..<7 {
                XCTAssertEqual(0, grid[float3(Float(i), 4, Float(k))]) // top slice all 0s

                if i == 3 && k == 3 { // next is all zeros except i,k=0
                    XCTAssertEqual(1, grid[float3(Float(i), 3, Float(k))])
                } else {
                    XCTAssertEqual(0, grid[float3(Float(i), 3, Float(k))])
                }

                // the rest decays like a pyramid:
                if (2...4).contains(i) && (2...4).contains(k) {
                    XCTAssertEqual(1/2, grid[float3(Float(i), 2, Float(k))])
                } else {
                    XCTAssertEqual(0, grid[float3(Float(i), 2, Float(k))])
                }

                if (1...5).contains(i) && (1...5).contains(k) {
                    XCTAssertEqual(1/4, grid[float3(Float(i), 1, Float(k))])
                } else {
                    XCTAssertEqual(0, grid[float3(Float(i), 1, Float(k))])
                }

                XCTAssertEqual(0, grid[float3(Float(i), 0, Float(k))])
            }
        }
    }
}
