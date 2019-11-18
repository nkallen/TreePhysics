import Foundation
import XCTest
@testable import TreePhysics
import simd

class AutoTreeTests: XCTestCase {
    func testGrow() throws {
        var config = AutoTree.Config()
        config.branchStraightnessBias = 0
        config.deflectionAngle = 0
        let autoTree = AutoTree(config)
        let (r, firstBud) = autoTree.seedling()

        // start with root -> terminalBud
        // transition to root -> internode -> terminalBud
        var (internode, (terminalBud, lateralBud)) = firstBud.grow(towards: [simd_float3(1,1,0), simd_float3(1,-1,0)])

        XCTAssertEqual(internode.position, firstBud.position)
        XCTAssertEqual(simd_quatf(angle: -.pi/2, axis: .z), internode.orientation, accuracy: 0.0001)
        XCTAssertNil(lateralBud)

        XCTAssertEqual(internode.orientation.heading * config.internodeLength, terminalBud.position, accuracy: 0.0001)
        XCTAssertEqual(internode.orientation, terminalBud.orientation)

        // transition to root -> internode -> [lateralBud] internode -> terminalBud
        (internode, (terminalBud, lateralBud)) = terminalBud.grow(towards: [simd_float3(10,0,0)])
        var lateralBud_ = try XCTUnwrap(lateralBud)

        XCTAssertEqual(internode.orientation.heading * config.internodeLength * 2, terminalBud.position, accuracy: 0.0001)
        XCTAssertEqual(
            simd_quatf(angle: config.phyllotacticAngle, axis: internode.orientation.heading) *
                internode.orientation,
            terminalBud.orientation)

        XCTAssertEqual(internode.position, lateralBud_.position)
        XCTAssertEqual(
            (simd_quatf(angle: config.phyllotacticAngle, axis: internode.orientation.heading) *
                simd_quatf(angle: -config.branchingAngle, axis: internode.orientation.up) *
                    internode.orientation).normalized,
            lateralBud_.orientation, accuracy: 0.0001)

        // transition to root -> internode -> [lateralBud] internode -> [lateralBud] internode -> terminalBud
        let branchingPoint = internode
        (internode, (terminalBud, lateralBud)) = terminalBud.grow(towards: [simd_float3(10,0,0)])
        lateralBud_ = try XCTUnwrap(lateralBud)

        XCTAssertEqual(internode.orientation.heading * config.internodeLength * 3, terminalBud.position, accuracy: 0.0001)
        XCTAssertEqual(
            simd_quatf(angle: config.phyllotacticAngle, axis: internode.orientation.heading) *
                internode.orientation,
            terminalBud.orientation, accuracy: 0.0001)

        XCTAssertEqual(internode.position, lateralBud_.position)
        XCTAssertEqual(
            (simd_quatf(angle: config.phyllotacticAngle, axis: branchingPoint.orientation.heading) *
                simd_quatf(angle: -config.branchingAngle, axis: branchingPoint.orientation.up) *
                    branchingPoint.orientation).normalized,
            lateralBud_.orientation, accuracy: 0.0001)
    }

    func testGrowWithStraighness() {
        var config = AutoTree.Config()
        config.branchStraightnessBias = 0.5
        config.branchGravitropismBias = 0
        let autoTree = AutoTree(config)
        let (root, firstBud) = autoTree.seedling()

        let (internode, (terminalBud, _)) = firstBud.grow(towards: [simd_float3(1,1,0), simd_float3(1,-1,0)])

        XCTAssertEqual(simd_quatf(angle: -.pi/4, axis: .z), internode.orientation, accuracy: 0.0001)

        XCTAssertEqual(internode.orientation.heading * config.internodeLength, terminalBud.position, accuracy: 0.0001)
    }

    func testGrowthWithGravitropism() {
        var config = AutoTree.Config()
        config.branchStraightnessBias = 0
        config.branchGravitropismBias = 0.5
        let autoTree = AutoTree(config)
        let (root, firstBud) = autoTree.seedling()

        let (internode, (terminalBud, _)) = firstBud.grow(towards: [simd_float3(1,1,0), simd_float3(1,-1,0)])

        XCTAssertEqual(simd_quatf(angle: -.pi/4, axis: .z), internode.orientation, accuracy: 0.0001)

        XCTAssertEqual(internode.orientation.heading * config.internodeLength, terminalBud.position, accuracy: 0.0001)
    }

    func testTerminalBudCount() throws {
        let config = AutoTree.Config()
        let autoTree = AutoTree(config)
        let (root, firstBud) = autoTree.seedling()

        XCTAssertEqual(1, root.terminalBudCount)
        let (_, (terminalBud0, _)) = firstBud.grow()
        XCTAssertEqual(1, root.terminalBudCount)
        let (_, (terminalBud1, lateralBud_)) = terminalBud0.grow()
        let lateralBud = try XCTUnwrap(lateralBud_)
        _ = terminalBud1.grow()
        _ = lateralBud.grow()
        XCTAssertEqual(2, root.terminalBudCount)
    }

    func testInternodeDiameter() {
        // FIXME
    }

    func testShadowGrid() {
        var config = AutoTree.Config()
        config.internodeLength = 1
        config.shadowDepth = 3
        config.initialShadowGridSize = 1 // This SHOULD trigger resizing
        let grid = AutoTree.ArrayBackedShadowGrid(config)
        grid[simd_float3(3.1,3.1,3.1)] += 1
        for i in 0..<8 {
            for k in 0..<7 {
                XCTAssertEqual(0, grid[simd_float3(Float(i), 4, Float(k))]) // top slice all 0s

                if i == 3 && k == 3 { // next is all zeros except i,k=0
                    XCTAssertEqual(1, grid[simd_float3(Float(i), 3, Float(k))])
                } else {
                    XCTAssertEqual(0, grid[simd_float3(Float(i), 3, Float(k))])
                }

                // the rest decays like a pyramid:
                if (2...4).contains(i) && (2...4).contains(k) {
                    XCTAssertEqual(1/2, grid[simd_float3(Float(i), 2, Float(k))])
                } else {
                    XCTAssertEqual(0, grid[simd_float3(Float(i), 2, Float(k))])
                }

                if (1...5).contains(i) && (1...5).contains(k) {
                    XCTAssertEqual(1/4, grid[simd_float3(Float(i), 1, Float(k))])
                } else {
                    XCTAssertEqual(0, grid[simd_float3(Float(i), 1, Float(k))])
                }

                XCTAssertEqual(0, grid[simd_float3(Float(i), 0, Float(k))])
            }
        }
    }

    func testVigorSimple() {
        let config = AutoTree.Config()
        let autoTree = AutoTree(config)
        let (root, firstBud) = autoTree.seedling()

        let (internode, (terminalBud, _)) = firstBud.grow()

        let fakeShadowGrid = FakeShadowGrid()
        let simulator = autoTree.growthSimulator(shadowGrid: fakeShadowGrid)
        simulator.addRoot(root)

        fakeShadowGrid[terminalBud.position] = config.shadowIntensity
        let exposures = simulator.updateLightExposure()
        let q = pow(config.fullExposure, config.sensitivityOfBudsToLight)
        XCTAssertEqual(q, exposures[internode])
        let vigors = simulator.updateVigor(exposures: exposures)
        XCTAssertEqual(q, vigors[internode])
    }

    func testVigorBranch() throws {
        let config = AutoTree.Config()
        let autoTree = AutoTree(config)
        let (root, firstBud) = autoTree.seedling()

        let (internode0, (terminalBud0, _)) = firstBud.grow()
        let (internode1, (terminalBud1, lateralBud1_)) = terminalBud0.grow()
        let lateralBud1 = try XCTUnwrap(lateralBud1_)
        let (internode2, (terminalBud2, _)) = lateralBud1.grow()

        let fakeShadowGrid = FakeShadowGrid()
        let simulator = autoTree.growthSimulator(shadowGrid: fakeShadowGrid)
        simulator.addRoot(root)

        fakeShadowGrid[terminalBud1.position] = config.shadowIntensity
        fakeShadowGrid[terminalBud2.position] = config.shadowIntensity * 4

        let exposures = simulator.updateLightExposure()

        let exposure0 = try XCTUnwrap(exposures[internode0])
        let exposure1 = try XCTUnwrap(exposures[internode1])
        let exposure2 = try XCTUnwrap(exposures[internode2])

        XCTAssertEqual(pow(config.fullExposure, config.sensitivityOfBudsToLight), exposure1)
        XCTAssertEqual(pow(config.fullExposure - config.shadowIntensity * 3, config.sensitivityOfBudsToLight), exposure2, accuracy: 0.0001)
        XCTAssertEqual(exposure1 + exposure2, exposure0)

        let vigors = simulator.updateVigor(exposures: exposures)

        let denominator: Float = (config.apicalDominance * exposure1 + (1 - config.apicalDominance) * exposure2) / exposure0

        XCTAssertEqual(exposure0, vigors[internode0])
        XCTAssertEqual(config.apicalDominance * exposure1 / denominator, vigors[internode1])
        XCTAssertEqual((1-config.apicalDominance) * exposure2 / denominator, vigors[internode2])
    }

    func testGravimorphismFactor() throws {
        var config = AutoTree.Config()
        config.horizontalGravimorphismBias = 1
        config.verticalGravimorphismBias = 10
        let autoTree = AutoTree(config)

        let internode0 = autoTree.internode(position: .zero, orientation: .x)
        let internode1 = autoTree.internode(position: .zero, orientation: .x)
        let verticalBud = autoTree.lateralBud(position: .zero, orientation: .y)
        let horizontalBud = autoTree.lateralBud(position: .zero, orientation: .z)
        internode0.addBud(horizontalBud)
        internode1.addBud(verticalBud)

        XCTAssertEqual(config.horizontalGravimorphismBias, horizontalBud.gravimorphismFactor)
        XCTAssertEqual(config.verticalGravimorphismBias, verticalBud.gravimorphismFactor, accuracy: 0.001)
    }
}

class FakeShadowGrid: AutoTree.ShadowGrid {
    var data: [simd_float3:Float] = [:]

    subscript(position: simd_float3) -> Float {
        get {
            return data[position] ?? 0
        }
        set(newValue) {
            data[position] = newValue
        }
    }


}
