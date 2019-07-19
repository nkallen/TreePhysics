import XCTest
@testable import TreePhysics
import simd

class SimulatorTests: XCTestCase {
    var simulator: Simulator!
    var root: RigidBody!
    var b1: RigidBody!
    var b2: RigidBody!
    let force = float2(0, 1) // world coordinates

    override func setUp() {
        super.setUp()

        root = RigidBody()
        b1 = RigidBody()
        b2 = RigidBody()
        root.add(b1, at: -Float.pi/4)
        b1.add(b2, at: -Float.pi/4)

        simulator = Simulator(tree: Tree(root))

        b2.apply(force: force, at: 1) // ie at float2(0, 1) in local coordinates
    }

    func testApplyForce() {
        XCTAssertEqual(b2.mass, 1)
        XCTAssertEqual(b2.force, force)

        XCTAssertEqual(b2.torque, cross(float2(0, 1), force))
        XCTAssertEqual(b2.momentOfInertia, 1.0/12 * 1 * 1) // moment of inertia is relative to center of mass

        XCTAssertEqual(root.position, float2(0,0))
        XCTAssertEqual(b1.parentJoint!.position, float2(0,1))
        XCTAssertEqual(b1.position, float2(0,1))

        XCTAssertEqual(b2.centerOfMass, float2(0.5 + 1/sqrt(2), 1 + 1/sqrt(2)), accuracy: 0.0001)
        XCTAssertEqual(b1.centerOfMass, float2(0.5/sqrt(2), 1 + 0.5/sqrt(2)), accuracy: 0.0001)
        XCTAssertEqual(root.centerOfMass, float2(0, 0.5), accuracy: 0.0001)

        XCTAssertEqual(b2.parentJoint!.position, float2(1/sqrt(2), 1 + 1/sqrt(2)))
        XCTAssertEqual(b1.parentJoint!.position, float2(0,1))
    }

    func testComposite() {
        simulator.update(at: 0)

        let forceAppliedPosition = b2.convert(position: float2(0, 1))

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

        XCTAssertEqual(Tree(root).flatten,
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

func XCTAssertEqual(_ x: float2, _ y: float2, accuracy: Float, file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(x.x, y.x, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(x.y, y.y, accuracy: accuracy, file: file, line: line)
}

class DifferentialTests: XCTestCase {
    func testRealDistinct() {
        let actual = solve_differential(a: 1, b: 11, c: 24, g: 0, y_0: 0, y_ddt_0: -7)
        let expected = DifferentialSolution.realDistinct(c1: -7.0/5, c2: 7.0/5, r1: -3, r2: -8, k: 0)
        XCTAssertEqual(actual, expected)
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
