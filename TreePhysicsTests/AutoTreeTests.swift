import Foundation
import XCTest
@testable import TreePhysics
import simd

class AutoTreeTests: XCTestCase {
    var config: AutoTreeConfig!
    var autoTree: AutoTree!
    var root: AutoTree.Parent!
    var firstBud: AutoTree.TerminalBud!

    override func setUp() {
        super.setUp()
        self.config = AutoTreeConfig()
        self.autoTree = AutoTree(config)
        let (root, firstBud) = autoTree.seedling()
        self.root = root
        self.firstBud = firstBud
    }

    func testGrow() throws {
        // start with root -> terminalBud
        // transition to root -> internode -> terminalBud
        var (internode, (terminalBud, lateralBud)) = firstBud.grow(towards: [float3(1,1,0), float3(1,-1,0)])

        XCTAssertEqual(internode.position, firstBud.position)
        XCTAssertEqual(simd_quatf(angle: -.pi/2, axis: .z), internode.orientation, accuracy: 0.0001)
        XCTAssertNil(lateralBud)

        XCTAssertEqual(.x * config.internodeLength, terminalBud.position, accuracy: 0.0001)
        XCTAssertEqual(simd_quatf(angle: config.phyllotacticAngle, axis: internode.orientation.heading) * internode.orientation, terminalBud.orientation)

        // transition to root -> internode -> [lateralBud] internode -> terminalBud
        (internode, (terminalBud, lateralBud)) = terminalBud.grow(towards: [float3(10,0,0)])
        let lateralBud_ = try XCTUnwrap(lateralBud)

        XCTAssertEqual(.x * config.internodeLength * 2, terminalBud.position, accuracy: 0.0001)
        XCTAssertEqual(
            (simd_quatf(angle: config.phyllotacticAngle, axis: internode.orientation.heading) * internode.orientation).normalized,
            terminalBud.orientation)

        XCTAssertEqual(internode.position, lateralBud_.position)
        XCTAssertEqual(
            (simd_quatf(angle: config.branchingAngle, axis: internode.orientation.up) * internode.orientation).normalized,
            lateralBud_.orientation)
    }

    // FIXME rename terminalBudCount
    func testTerminalBranchCount() throws {
        XCTAssertEqual(1, root.terminalBranchCount)
        let (_, (terminalBud0, _)) = firstBud.grow()
        XCTAssertEqual(1, root.terminalBranchCount)
        let (_, (terminalBud1, lateralBud_)) = terminalBud0.grow()
        let lateralBud = try XCTUnwrap(lateralBud_)
        _ = terminalBud1.grow()
        _ = lateralBud.grow()
        XCTAssertEqual(2, root.terminalBranchCount)
    }

    func testShadowGrid() {
        let config = AutoTree.ShadowGridConfig(cellSize: 1)
        let grid = AutoTree.HashingShadowGrid(config)
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

    func testVigorSimple() {
        let (internode, (terminalBud, _)) = firstBud.grow()

        let fakeShadowGrid = FakeShadowGrid()
        let simulator = autoTree.growthSimulator(shadowGrid: fakeShadowGrid)
        simulator.add(root)

        fakeShadowGrid[terminalBud.position] = config.shadowIntensity
        let exposures = simulator.updateLightExposure()
        XCTAssertEqual(config.fullExposure, exposures[internode])
        let vigors = simulator.updateVigor(exposures: exposures)
        XCTAssertEqual(pow(config.fullExposure, config.k), vigors[internode])
    }

    func testVigorBranch() throws {
        let (internode0, (terminalBud0, _)) = firstBud.grow()
        let (internode1, (terminalBud1, lateralBud1_)) = terminalBud0.grow()
        let lateralBud1 = try XCTUnwrap(lateralBud1_)
        let (internode2, (terminalBud2, _)) = lateralBud1.grow()

        let fakeShadowGrid = FakeShadowGrid()
        let simulator = autoTree.growthSimulator(shadowGrid: fakeShadowGrid)
        simulator.add(root)

        fakeShadowGrid[terminalBud1.position] = config.shadowIntensity
        fakeShadowGrid[terminalBud2.position] = config.shadowIntensity * 4

        let exposures = simulator.updateLightExposure()

        let exposure0 = try XCTUnwrap(exposures[internode0])
        let exposure1 = try XCTUnwrap(exposures[internode1])
        let exposure2 = try XCTUnwrap(exposures[internode2])

        XCTAssertEqual(config.fullExposure, exposure1)
        XCTAssertEqual(config.fullExposure - config.shadowIntensity * 3, exposure2)
        XCTAssertEqual(exposure1 + exposure2, exposure0)

        let vigors = simulator.updateVigor(exposures: exposures)
        let v = pow(exposure0, config.k)
        let qm = pow(exposure1, config.k)
        let ql = pow(exposure2, config.k)

        let denominator: Float = (config.lambda * qm + (1 - config.lambda) * ql) / v

        XCTAssertEqual(v, vigors[internode0])
        XCTAssertEqual(config.lambda * qm / denominator, vigors[internode1])
        XCTAssertEqual((1-config.lambda) * ql / denominator, vigors[internode2])
    }
}

class FakeShadowGrid: AutoTree.ShadowGrid {
    var data: [float3:Float] = [:]

    subscript(position: float3) -> Float {
        get {
            print("asked about \(position) and returned \(data[position] ?? 0)")
            return data[position] ?? 0
        }
        set(newValue) {
            data[position] = newValue
        }
    }


}
