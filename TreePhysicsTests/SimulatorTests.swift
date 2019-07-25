import XCTest
@testable import TreePhysics
import simd

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
        root.add(b1, at: -Float.pi/4)
        b1.add(b2, at: -Float.pi/4)

        simulator = Simulator(tree: Tree(root))

        b2.apply(force: force, at: 1) // ie at float3(0, 1, 0) in local coordinates
    }

    func testInertiaTensorsInCompositeBodies() {
        let start = RigidBody()
        let stop = RigidBody()
        start.add(stop, at: 0)
        let simulator = Simulator(tree: Tree(start))
        simulator.updateCompositeBodies()

        XCTAssertEqual(start.momentOfInertia, 1.0/12)
        XCTAssertEqual(stop.momentOfInertia, 1.0/12)

        XCTAssertEqual(stop.composite.momentOfInertia, 1.0/12)
        XCTAssertEqual(start.composite.momentOfInertia, Float((1.0/12 + 1.0/4) * 2.0))

        let rotation_world2local_stop = matrix3x3_rotation(from: matrix_float4x4(diagonal: float4(1,1,1,0)), to: stop.transform)
        let rotation_world2local_start = matrix3x3_rotation(from: matrix_float4x4(diagonal: float4(1,1,1,0)), to: start.transform)

        XCTAssertEqual((rotation_world2local_stop * stop.inertiaTensor * rotation_world2local_stop.transpose * float3(0,0,1)).z, 1.0/12 + 1.0/4, accuracy: 0.0001)
        XCTAssertEqual((rotation_world2local_start * start.inertiaTensor * rotation_world2local_start.transpose * float3(0,0,1)).z, 1.0/12 + 1.0/4, accuracy: 0.0001)

        XCTAssertEqual((rotation_world2local_stop * stop.composite.inertiaTensor * rotation_world2local_stop.transpose * float3(0,0,1)).z, 1.0/12 + 1.0/4, accuracy: 0.0001)
        XCTAssertEqual((rotation_world2local_start * start.composite.inertiaTensor * rotation_world2local_start.transpose * float3(0,0,1)).z,
                       Float(1.0/12 * 2 * 2*2 + 1.0/4 * 2 * 1*1), accuracy: 0.0001)

        print(rotation_world2local_start * start.composite.inertiaTensor * rotation_world2local_start.transpose , Float((1.0/12 + 1.0/4) * 2.0))
    }

    func testApplyForce() {
        XCTAssertEqual(b2.mass, 1)
        XCTAssertEqual(b2.force, force)
        XCTAssertEqual(b2.length, 1)
        XCTAssertEqual(b2.radius, 1)

        XCTAssertEqual(b2.torque, cross(b2.convert(position: float3(0, 1, 0)) - b2.position, force))
        XCTAssertEqual(b2.momentOfInertia, 1.0/12 * 1 * 1) // moment of inertia is relative to center of mass
        let rotation_world2local = matrix3x3_rotation(from: matrix_float4x4(diagonal: float4(1,1,1,0)), to: b2.transform)
        XCTAssertEqual(rotation_world2local * b2.inertiaTensor * rotation_world2local.transpose, matrix_float3x3.init(diagonal: float3(
            1.0/4 + 1.0/12,
            1.0/2,
            1.0/4 + 1.0/12
        )), accuracy: 0.0001)

        XCTAssertEqual(root.position, float3.zero)
        XCTAssertEqual(b1.parentJoint!.position, float3(0,1,0))
        XCTAssertEqual(b1.position, float3(0,1,0))

        XCTAssertEqual(b2.centerOfMass, float3(0.5 + 1/sqrt(2), 1 + 1/sqrt(2), 0), accuracy: 0.0001)
        XCTAssertEqual(b1.centerOfMass, float3(0.5/sqrt(2), 1 + 0.5/sqrt(2), 0), accuracy: 0.0001)
        XCTAssertEqual(root.centerOfMass, float3(0, 0.5, 0), accuracy: 0.0001)

        XCTAssertEqual(b2.parentJoint!.position, float3(1/sqrt(2), 1 + 1/sqrt(2), 0), accuracy: 0.0001)
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

        // inertia
        XCTAssertEqual(b2.composite.momentOfInertia, b2.momentOfInertia)
        XCTAssertEqual(b1.composite.momentOfInertia,
                       b1.momentOfInertia + b1.mass * square(distance(b1.centerOfMass, b1.composite.centerOfMass)) +
                        b2.momentOfInertia + b2.mass * square(distance(b2.centerOfMass, b1.composite.centerOfMass)),
                       accuracy: 0.0001)
        XCTAssertEqual(root.composite.momentOfInertia,
                       root.momentOfInertia + root.mass * square(distance(root.centerOfMass, root.composite.centerOfMass)) +
                        b1.momentOfInertia + b1.mass * square(distance(b1.centerOfMass, root.composite.centerOfMass)) +
                        b2.momentOfInertia + b2.mass * square(distance(b2.centerOfMass, root.composite.centerOfMass)),
                       accuracy: 0.0001)
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

        XCTAssertEqual(root.flatten,
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

func XCTAssertEqual(_ a: float3, _ b: float3, accuracy: Float, file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.x, b.x, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.y, b.y, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.z, b.z, accuracy: accuracy, file: file, line: line)
}

func XCTAssertEqual(_ a: float3x3, _ b: float3x3, accuracy: Float, file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.columns.0, b.columns.0, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.columns.1, b.columns.1, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.columns.2, b.columns.2, accuracy: accuracy, file: file, line: line)
}

func XCTAssertEqual(_ a: [float3], _ b: [float3], accuracy: Float, file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.count, b.count, file: file, line: line)
    for (left, right) in zip(a, b) {
        XCTAssertEqual(left, right, accuracy: accuracy, file: file, line: line)
    }
}

func XCTAssertEqual(_ a: DifferentialSolution, _ b: DifferentialSolution, accuracy: Double, file: StaticString = #file, line: UInt = #line) {
    switch (a, b) {
    case let (.realDistinct(c1_left, c2_left, r1_left, r2_left, k_left),
              .realDistinct(c1_right, c2_right, r1_right, r2_right, k_right)):
        XCTAssertEqual(c1_left, c1_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(c2_left, c2_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(r1_left, r1_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(r2_left, r2_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(k_left, k_right, accuracy: accuracy)
    case let (.real(c1_left, c2_left, r_left, k_left),
              .real(c1_right, c2_right, r_right, k_right)):
        XCTAssertEqual(c1_left, c1_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(c2_left, c2_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(r_left, r_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(k_left, k_right, accuracy: accuracy, file: file, line: line)
    case let (.complex(c1_left, c2_left, lambda_left, mu_left, k_left),
          .complex(c1_right, c2_right, lambda_right, mu_right, k_right)):
        XCTAssertEqual(c1_left, c1_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(c2_left, c2_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(lambda_left, lambda_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(mu_left, mu_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(k_left, k_right, accuracy: accuracy, file: file, line: line)
    default:
        XCTFail(file: file, line: line)
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
        let expected = DifferentialSolution.complex(c1: 0, c2: -8/sqrt(5), lambda: 2, mu: sqrt(5), k: 0)
        XCTAssertEqual(actual, expected)
        XCTAssertEqual(evaluate(differential: actual, at: 0), float3(0, -8, -32))
    }
}
