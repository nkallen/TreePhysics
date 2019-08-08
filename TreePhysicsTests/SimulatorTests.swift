import XCTest
@testable import TreePhysics
import simd

fileprivate let sqrt2: Float = sqrtf(2)

class SimulatorTests: XCTestCase {
    var simulator: Simulator!
    var root: RigidBody!
    var b1: RigidBody!
    var b2: RigidBody!
    let force = float3(0, 1, 0) // world coordinates

    override func setUp() {
        super.setUp()

        root = RigidBody()
        b1 = RigidBody()
        b2 = RigidBody()
        root.add(b1, at: float3(0,0,-Float.pi/4))
        b1.add(b2, at: float3(0,0,-Float.pi/4))

        simulator = Simulator(tree: Tree(root))

        b2.apply(force: force, at: 1) // ie at float3(0, 1, 0) in local coordinates
    }

    func testInertiaTensorsInCompositeBodies() {
        let start = RigidBody()
        let stop = RigidBody()
        start.add(stop, at: float3.zero)
        let simulator = Simulator(tree: Tree(start))
        simulator.updateCompositeBodies()

        XCTAssertEqual(start.inertiaTensor, float3x3([[0.3333, 0, 0], [0, 0.5, 0], [0, 0, 0.3333]]), accuracy: 0.0001)
        XCTAssertEqual(stop.inertiaTensor, float3x3([[0.3333, 0, 0], [0, 0.5, 0], [0, 0, 0.3333]]), accuracy: 0.0001)

        XCTAssertEqual(stop.composite.inertiaTensor, float3x3([[0.3333, 0, 0], [0, 0.5, 0], [0, 0, 0.3333]]), accuracy: 0.0001)
        XCTAssertEqual(start.composite.inertiaTensor, float3x3([[1.1666667, 0, 0], [0, 1, 0], [0, 0, 1.1666667]]), accuracy: 0.0001)

        let rotation_world2local_stop = matrix3x3_rotation(from: matrix_float4x4(diagonal: float4(1,1,1,0)), to: stop.transform)
        let rotation_world2local_start = matrix3x3_rotation(from: matrix_float4x4(diagonal: float4(1,1,1,0)), to: start.transform)

        XCTAssertEqual((rotation_world2local_stop * stop.inertiaTensor * rotation_world2local_stop.transpose * float3(0,0,1)).z, 1.0/12 + 1.0/4, accuracy: 0.0001)
        XCTAssertEqual((rotation_world2local_start * start.inertiaTensor * rotation_world2local_start.transpose * float3(0,0,1)).z, 1.0/12 + 1.0/4, accuracy: 0.0001)

        XCTAssertEqual((rotation_world2local_stop * stop.composite.inertiaTensor * rotation_world2local_stop.transpose * float3(0,0,1)).z, 1.0/12 + 1.0/4, accuracy: 0.0001)
        XCTAssertEqual((rotation_world2local_start * start.composite.inertiaTensor * rotation_world2local_start.transpose * float3(0,0,1)).z,
                       Float(1.0/12 * 2 * 2*2 + 1.0/4 * 2 * 1*1), accuracy: 0.0001)
    }

    func testTransform() {
        XCTAssertEqual(b2.inertiaTensor,
                       float3x3([[0.5, -2.4835266e-08, 0], [-2.4835263e-08, 0.33333328, 0], [0, 0, 0.3333]]), accuracy: 0.0001)
        XCTAssertEqual(b2.transform,
                       float4x4([[1.4901161e-07, -1, 0, 0], [1, 1.4901161e-07, 0, 0], [0, 0, 1, 0], [1/sqrt2, 1.7071068, 0, 1]]), accuracy: Float(0.0001))
        XCTAssertEqual(b2.parentJoint!.transform, float4x4([[1/sqrt2, -1/sqrt2, 0, 0], [1/sqrt2, 1/sqrt2, 0, 0], [0, 0, 1, 0], [1/sqrt2, 1.7071068, 0, 1]]), accuracy: 0.0001)
        XCTAssertEqual(b1.inertiaTensor,
                       float3x3([[0.41666663, -0.08333331, 0], [-0.08333333, 0.4166667, 0], [0, 0, 0.3333]]),
                       accuracy: 0.0001)
        XCTAssertEqual(b1.transform,
                       float4x4([[1/sqrt2, -1/sqrt2, 0, 0], [1/sqrt2, 1.0/sqrt2, 0, 0], [0, 0, 1, 0], [0, 1, 0, 1]]),
                       accuracy: 0.0001)
        XCTAssertEqual(b1.parentJoint!.transform,
                       float4x4([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 1, 0, 1]]), accuracy: 0.0001)

        simulator.update(at: 1.0 / 60)

        XCTAssertEqual(b2.inertiaTensor,
                       float3x3([[0.49996945, -0.0022556656, 0], [-0.0022556656, 0.33336386, 0], [0, 0, 0.3333]]), accuracy: 0.0001)
        XCTAssertEqual(b2.transform,
                       float4x4([[0.013535231, -0.9999084, 0, 0], [0.9999084, 0.013535231, 0, 0], [0, 0, 1, 0], [0.7010455, 1.7131165, 0, 1]]), accuracy: Float(0.0001))
        XCTAssertEqual(b2.parentJoint!.transform, float4x4([[0.7166128, -0.6974712, 0, 0], [0.6974712, 0.7166128, 0, 0], [0, 0, 1, 0], [0.7010455, 1.7131165, 0, 1]]), accuracy: 0.0001)

        XCTAssertEqual(b1.inertiaTensor,
                       float3x3([[0.41524416, -0.083321184, 0], [-0.083321184, 0.4180892, 0], [0, 0, 0.3333]]),
                       accuracy: 0.0001)
        XCTAssertEqual(b1.transform,
                       float4x4([[0.7131165, -0.7010455, 0, 0], [0.7010455, 0.7131165, 0, 0], [0, 0, 1, 0], [0, 1, 0, 1]]),
                       accuracy: 0.0001)
        XCTAssertEqual(b1.parentJoint!.transform,
                       float4x4([[0.9999636, 0.008535428, 0, 0], [-0.008535428, 0.9999636, 0, 0], [0, 0, 1, 0], [0, 1, 0, 1]]), accuracy: 0.0001)
    }

    func testApplyForce() {
        XCTAssertEqual(b2.mass, 1)
        XCTAssertEqual(b2.force, force)
        XCTAssertEqual(b2.length, 1)
        XCTAssertEqual(b2.radius, 1)

        XCTAssertEqual(b2.torque, cross(b2.convert(position: float3(0, 1, 0)) - b2.position, force))

        let rotation_world2local = matrix3x3_rotation(from: matrix_float4x4(diagonal: float4(1,1,1,0)), to: b2.transform)
        XCTAssertEqual(rotation_world2local * b2.inertiaTensor * rotation_world2local.transpose, matrix_float3x3.init(diagonal: float3(
            1.0/4 + 1.0/12,
            1.0/2,
            1.0/4 + 1.0/12
        )), accuracy: 0.0001)

        XCTAssertEqual(root.position, float3.zero)
        XCTAssertEqual(b1.parentJoint!.position, float3(0,1,0))
        XCTAssertEqual(b1.position, float3(0,1,0))

        XCTAssertEqual(b2.centerOfMass, float3(0.5 + 1/sqrt2, 1 + 1/sqrt2, 0), accuracy: 0.0001)
        XCTAssertEqual(b1.centerOfMass, float3(0.5/sqrt2, 1 + 0.5/sqrt2, 0), accuracy: 0.0001)
        XCTAssertEqual(root.centerOfMass, float3(0, 0.5, 0), accuracy: 0.0001)

        XCTAssertEqual(b2.parentJoint!.position, float3(1/sqrt2, 1 + 1/sqrt2, 0), accuracy: 0.0001)
        XCTAssertEqual(b1.parentJoint!.position, float3(0,1, 0))
    }

    func testComposite() {
        simulator.updateCompositeBodies()

        let forceAppliedPosition = b2.convert(position: float3(0, 1, 0))

        // mass
        XCTAssertEqual(b2.composite.mass, 1)
        XCTAssertEqual(b1.composite.mass, 2)
        XCTAssertEqual(root.composite.mass, 3)

        // force
        XCTAssertEqual(b2.composite.force, force)
        XCTAssertEqual(b1.composite.force, force)
        XCTAssertEqual(root.composite.force, force)

        // torque
        XCTAssertEqual(b2.composite.torque, b2.torque)
        let r_b1 = forceAppliedPosition - b1.parentJoint!.position
        XCTAssertEqual(b1.composite.torque, cross(r_b1, force))
        let r_root = forceAppliedPosition - root.position
        XCTAssertEqual(root.composite.torque, cross(r_root, force))

        // center of mass
        XCTAssertEqual(b2.composite.centerOfMass, b2.centerOfMass)
        XCTAssertEqual(b1.composite.centerOfMass, (b1.centerOfMass + b2.centerOfMass)/2)
        XCTAssertEqual(root.composite.centerOfMass, (b1.centerOfMass + b2.centerOfMass + root.centerOfMass) / 3)
    }
}

class TreeTests: XCTestCase {
    func testFlattenIsBreadthFirst() {
        let root = RigidBody()
        let b1 = RigidBody()
        let b2 = RigidBody()
        let b3 = RigidBody()
        let b4 = RigidBody()
        let b5 = RigidBody()
        let b6 = RigidBody()
        let b7 = RigidBody()
        root.add(b1)
        root.add(b2)
        b1.add(b3)
        b1.add(b4)
        b4.add(b5)
        b2.add(b6)
        b2.add(b7)

        XCTAssertEqual(root.flattened,
                       [root, b1, b2, b3, b4, b6, b7, b5])
    }
}

class QuadraticTests: XCTestCase {
    func testRealDistinct() {
        XCTAssertEqual(solve_quadratic(a: 1, b: 11, c: 24),
                       .realDistinct(-3, -8))
    }

    func testReal() {
        XCTAssertEqual(solve_quadratic(a: 1, b: -4, c: 4),
                       .real(2))
    }

    func testComplex() {
        XCTAssertEqual(solve_quadratic(a: 1, b: -4, c: 9),
                       .complex(2, sqrt(5)))
    }
}

class DifferentialTests: XCTestCase {
    func testRealDistinct() {
        let actual = solve_differential(a: 1, b: 11, c: 24, g: 0, y_0: 0, y_ddt_0: -7)
        let expected = DifferentialSolution.realDistinct(c1: -7.0/5, c2: 7.0/5, r1: -3, r2: -8, k: 0)
        XCTAssertEqual(actual, expected, accuracy: 0.0001)
        XCTAssertEqual(evaluate(differential: actual, at: 0), float3(0, -7, 77))
    }

    func testReal() {
        let actual = solve_differential(a: 1, b: -4, c: 4, g: 0, y_0: 12, y_ddt_0: -3)
        let expected = DifferentialSolution.real(c1: 12, c2: -27, r: 2, k: 0)
        XCTAssertEqual(actual, expected)
        XCTAssertEqual(evaluate(differential: actual, at: 0), float3(12, -3, -60))
    }

    func testComplex() {
        let actual = solve_differential(a: 1, b: -4, c: 9, g: 0, y_0: 0, y_ddt_0: -8)
        let expected = DifferentialSolution.complex(c1: 0, c2: -8/sqrt(5), λ: 2, μ: sqrt(5), k: 0)
        XCTAssertEqual(actual, expected)
        XCTAssertEqual(evaluate(differential: actual, at: 0), float3(0, -8, -32))
    }
}
