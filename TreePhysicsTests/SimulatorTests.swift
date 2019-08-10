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

        // inertia tensor
        XCTAssertEqual(b2.composite.inertiaTensor, b2.inertiaTensor)
        var b1_inertiaTensor = b1.inertiaTensor - b1.mass * sqr((b1.centerOfMass - b1.composite.centerOfMass).crossMatrix)
        b1_inertiaTensor += b2.composite.inertiaTensor - b2.composite.mass * sqr((b2.composite.centerOfMass - b1.composite.centerOfMass).crossMatrix)
        XCTAssertEqual(b1.composite.inertiaTensor, b1_inertiaTensor, accuracy: 0.0001)
        var root_inertiaTensor = root.inertiaTensor - root.mass * sqr((root.centerOfMass - root.composite.centerOfMass).crossMatrix)
        root_inertiaTensor += b1.composite.inertiaTensor - b1.composite.mass * sqr((b1.composite.centerOfMass - root.composite.centerOfMass).crossMatrix)
        XCTAssertEqual(root.composite.inertiaTensor, root_inertiaTensor)
    }

    func testUpdateJoints() {
        simulator.updateCompositeBodies()
        simulator.updateJoints(at: 1/60)

        XCTAssertEqual(
            float3x3(
                float3(0,0,0.005), // i.e., a small rotation about the z axis
                float3(0,0,0),
                float3(0,0,0)
            ),
            b2.parentJoint!.θ, accuracy: 0.0001)

        XCTAssertEqual(
            float3x3(
                float3(0,0,0.008535533), // a slightly larger rotation since the torque on b1 is greater
                float3(0,0,0),
                float3(0,0,0)
            ),
            b1.parentJoint!.θ, accuracy: 0.0001)
    }

    func testUpdateRigidBodies() {
        XCTAssertEqual(
            float3(1/sqrt2, 1+1/sqrt2, 0),
            b2.position)
        XCTAssertEqual(
            float3x3(
            float3(0,-1,0),
            float3(1,0,0),
            float3(0,0,1)),
            b2.rotation, accuracy: 0.0001)
        XCTAssertEqual(
            float4x4(
                [[1.4901161e-07, -1, 0, 0], [1, 1.4901161e-07, 0, 0], [0, 0, 1, 0], [1/sqrt2, 1+1/sqrt2, 0, 1]]),
            b2.transform, accuracy: Float(0.0001))

        XCTAssertEqual(
            float3(0, 1, 0),
            b1.position)
        XCTAssertEqual(
            float3x3(
                float3(1/sqrt(2),-1/sqrt2,0),
                float3(1/sqrt(2),1/sqrt(2),0),
                float3(0,0,1)),
            b1.rotation, accuracy: 0.0001)
        XCTAssertEqual(
            float4x4([[1/sqrt2, -1/sqrt2, 0, 0], [1/sqrt2, 1.0/sqrt2, 0, 0], [0, 0, 1, 0], [0, 1, 0, 1]]),
            b1.transform, accuracy: 0.0001)

        simulator.update(at: 1.0 / 60)

        XCTAssertEqual(
            float3(0.7010456, 1.7131165, 0),
            b2.position, accuracy: 0.0001)
        XCTAssertEqual(
            float3x3(
                float3(0.013535142,-1,0),
                float3(1,0.013535142,0),
                float3(0,0,1)),
            b2.rotation, accuracy: 0.0001)
        XCTAssertEqual(
            float4x4(
                [[0.013535231, -0.9999084, 0, 0], [0.9999084, 0.013535231, 0, 0], [0, 0, 1, 0], [0.7010455, 1.7131165, 0, 1]]),
            b2.transform, accuracy: Float(0.0001))

        XCTAssertEqual(
            float3(0, 1, 0),
            b1.position)
        XCTAssertEqual(
            float3x3(
                float3(0.7131165,-0.7010456,0),
                float3(0.7010456,0.7131165,0),
                float3(0,0,1)),
            b1.rotation, accuracy: 0.0001)
        XCTAssertEqual(
            float4x4([[0.7131165, -0.7010455, 0, 0], [0.7010455, 0.7131165, 0, 0], [0, 0, 1, 0], [0, 1, 0, 1]]),
            b1.transform, accuracy: 0.0001)
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
