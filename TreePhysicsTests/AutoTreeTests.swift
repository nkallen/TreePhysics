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

    func testLightExposureSimple() {
        let root = autoTree.root()
        let internode = autoTree.internode(position: .zero, orientation: .identity)
        let bud = autoTree.terminalBud(position: .y * 1, orientation: .identity)

        let simulator = autoTree.growthSimulator()
        simulator.add(root)
        simulator.updateVigor()
        XCTAssertEqual(config.fullExposure, internode.lightExposure)
    }

    func testLightExposureBranch() {
        let root = autoTree.root()
        let internode0 = autoTree.internode(position: .zero, orientation: .identity)
        let internode1 = autoTree.internode(position: .y * 1, orientation: .identity)
        let internode2 = autoTree.internode(position: .y * 1, orientation: simd_quatf(angle: .pi/4, axis: .z))
        let bud1 = autoTree.terminalBud(position: internode1.position + internode1.orientation.heading * 1, orientation: .identity)
        let bud2 = autoTree.terminalBud(position: internode2.position + internode2.orientation.heading * 1, orientation: .identity)

        root.addChild(internode0)
        internode0.addChild(internode1)
        internode0.addChild(internode2)
        internode1.addChild(bud1)
        internode2.addChild(bud2)

        let simulator = autoTree.growthSimulator()
        simulator.add(root)
        simulator.updateVigor()

        XCTAssertEqual(config.fullExposure, internode1.lightExposure)
        XCTAssertEqual(config.fullExposure, internode2.lightExposure)
        XCTAssertEqual(internode1.lightExposure + internode2.lightExposure, internode0.lightExposure)
    }
}
