import XCTest
@testable import TreePhysics
import simd

fileprivate let sqrt2: Float = sqrtf(2)
fileprivate let delta: TimeInterval = 1

/*
 The simplest test: One vertical node (attached to a root) with a force applied in the x direction.
 */
class SimpleCPUSimulatorTests: XCTestCase {
    var simulator: CPUSimulator!
    var b0: Internode!
    var joint: Joint!
    var forceAppliedPosition: float3!
    let force = SIMD3<Float>(1, 0, 0)

    override func setUp() {
        super.setUp()

        let root = ArticulatedRigidBody.static()
        b0 = Internode()
        joint = root.add(b0)

        let world = PhysicsWorld()
        simulator = CPUSimulator(world: world)
        world.add(rigidBody: root)

        b0.apply(force: force)
        self.forceAppliedPosition = b0.pivot + b0.rotation.act(SIMD3<Float>(0, 1/2, 0))
    }

    func testPreconditions() {
        XCTAssertEqual(b0.pivot, .zero)
        XCTAssertEqual(b0.centerOfMass, SIMD3<Float>(0,1/2,0))
        XCTAssertEqual(b0.mass, 1)
        XCTAssertEqual(b0.force, force)
        XCTAssertEqual(b0.torque, cross(forceAppliedPosition, force))
        XCTAssertEqual(b0.radius, 1)
        XCTAssertEqual(b0.length, 1)
        XCTAssertEqual(b0.torque, cross(forceAppliedPosition - b0.pivot, force), accuracy: 0.0001)
        XCTAssertEqual(float3x3(b0.rotation.normalized) * b0.inertiaTensor * float3x3(b0.rotation.normalized).transpose,
                       float3x3(diagonal: SIMD3<Float>(
                        1.0/4 + 1.0/12,
                        1.0/2,
                        1.0/4 + 1.0/12
                       )), accuracy: 0.0001)

        XCTAssertEqual(joint.stiffness, 1)
        XCTAssertEqual(joint.damping, 1)
    }

    func testComposite() {
        simulator.updateCompositeBodies()

        XCTAssertEqual(b0.composite.mass, b0.mass)
        XCTAssertEqual(b0.composite.force, b0.force)
        XCTAssertEqual(b0.composite.torque, b0.torque)
        XCTAssertEqual(b0.composite.centerOfMass, b0.centerOfMass)
        XCTAssertEqual(b0.composite.inertiaTensor, b0.inertiaTensor)
    }

    func testUpdateJoints() {
        simulator.updateCompositeBodies()
        simulator.updateJoints(at: delta)

        let momentOfInertia: Float = 1/4 + 1/12
        let compositeInertiaRelativeToJoint = momentOfInertia +
            b0.composite.mass * sqr(distance(b0.composite.centerOfMass, joint.position))

        let θ = evaluateDifferential(a: compositeInertiaRelativeToJoint, b: joint.damping * joint.stiffness, c: joint.stiffness, g: b0.composite.torque.z, y_0: 0, y_ddt_0: 0, at: Float(delta))

        XCTAssertEqual(
            float3x3(
                SIMD3<Float>(0,0,θ[0]),
                SIMD3<Float>(0,0,θ[1]),
                SIMD3<Float>(0,0,θ[2])
            ),
            b0.parentJoint!.θ, accuracy: 0.0001)
    }

    func testUpdateRigidBodies() {
        simulator.update(at: delta)

        let momentOfInertia: Float = 1/4 + 1/12
        let compositeInertiaRelativeToJoint = momentOfInertia +
            b0.composite.mass * sqr(distance(b0.composite.centerOfMass, joint.position))
        let θ = evaluateDifferential(a: compositeInertiaRelativeToJoint, b: joint.damping * joint.stiffness, c: joint.stiffness, g: b0.composite.torque.z, y_0: 0, y_ddt_0: 0, at: Float(delta))

        let rotation = simd_quatf(angle: θ[0], axis: .z)
        XCTAssertEqual(
            rotation.act(SIMD3<Float>(0, 0.5, 0)),
            b0.centerOfMass, accuracy: 0.0001)
        XCTAssertEqual(
            rotation,
            b0.rotation, accuracy: 0.0001)
    }
}

/*
 The second simplest test: One vertical node with a child at rotation -π/4 and a force applied in the x direction.
 */
class CPUSimulatorTests: XCTestCase {
    var simulator: CPUSimulator!
    var b0: Internode!
    var b1: Internode!
    let force = SIMD3<Float>(1, 0, 0) // world coordinates
    var forceAppliedPosition: float3!

    override func setUp() {
        super.setUp()

        let root = ArticulatedRigidBody.static()
        b0 = Internode()
        b1 = Internode()
        _ = root.add(b0)
        _ = b0.add(b1)

        let world = PhysicsWorld()
        simulator = CPUSimulator(world: world)
        world.add(rigidBody: root)

        b1.apply(force: force)
        self.forceAppliedPosition = b1.rotation.act(SIMD3<Float>(0, 1/2, 0))
    }

    func testPreconditions() {
        XCTAssertEqual(b0.pivot, .zero)
        XCTAssertEqual(b0.centerOfMass, SIMD3<Float>(0,1/2,0))
        XCTAssertEqual(b0.mass, 1)
        XCTAssertEqual(b0.force, .zero)
        XCTAssertEqual(b0.torque, .zero)
        XCTAssertEqual(b0.radius, 1)
        XCTAssertEqual(b0.length, 1)
        XCTAssertEqual(float3x3(b0.rotation).transpose * b0.inertiaTensor * float3x3(b0.rotation),
                       float3x3(diagonal: SIMD3<Float>(
                        1.0/4 + 1.0/12,
                        1.0/2,
                        1.0/4 + 1.0/12
                       )), accuracy: 0.0001)

        XCTAssertEqual(b0.parentJoint!.stiffness, 1)
        XCTAssertEqual(b0.parentJoint!.damping, 1)

        XCTAssertEqual(b1.pivot, SIMD3<Float>(0,1,0))
        XCTAssertEqual(b1.centerOfMass, b1.pivot + b1.rotation.act(SIMD3<Float>(0,1/2,0)), accuracy: 0.0001)
        XCTAssertEqual(b1.mass, 1)
        XCTAssertEqual(b1.force, force)
        XCTAssertEqual(b1.torque, cross(forceAppliedPosition, force))
        XCTAssertEqual(b1.radius, 1)
        XCTAssertEqual(b1.length, 1)
        XCTAssertEqual(float3x3(b1.rotation).transpose * b1.inertiaTensor * float3x3(b1.rotation),
                       float3x3(diagonal: SIMD3<Float>(
                        1.0/4 + 1.0/12,
                        1.0/2,
                        1.0/4 + 1.0/12
                       )), accuracy: 0.0001)

        XCTAssertEqual(b1.parentJoint!.stiffness, 1)
        XCTAssertEqual(b1.parentJoint!.damping, 1)
    }

    func testComposite() {
        simulator.updateCompositeBodies()

        XCTAssertEqual(b1.composite.mass, b1.mass)
        XCTAssertEqual(b0.composite.mass, b1.mass + b0.mass)

        XCTAssertEqual(b1.composite.force, b1.force)
        XCTAssertEqual(b0.composite.force, b1.force + b0.force)

        XCTAssertEqual(b1.composite.torque, b1.torque)
        XCTAssertEqual(b0.composite.torque, cross(b1.centerOfMass, force))

        XCTAssertEqual(b1.composite.centerOfMass, b1.centerOfMass)
        XCTAssertEqual(b0.composite.centerOfMass, (b1.centerOfMass + b0.centerOfMass) / 2)

        XCTAssertEqual(b1.composite.inertiaTensor, b1.inertiaTensor)
        var b0_inertiaTensor = b0.inertiaTensor - b0.mass * sqr((b0.centerOfMass - b0.composite.centerOfMass).skew)
        b0_inertiaTensor += b1.composite.inertiaTensor - b1.composite.mass * sqr((b1.composite.centerOfMass - b0.composite.centerOfMass).skew)
        XCTAssertEqual(b0.composite.inertiaTensor, b0_inertiaTensor, accuracy: 0.0001)
    }

    func testUpdateJoints() {
        simulator.updateCompositeBodies()
        simulator.updateJoints(at: delta)

        do {
            let joint = b1.parentJoint!

            let momentOfInertia: Float = 1/4 + 1/12
            let compositeInertiaRelativeToJoint = momentOfInertia +
                b1.composite.mass * sqr(distance(b1.composite.centerOfMass, joint.position))

            let θ = evaluateDifferential(a: compositeInertiaRelativeToJoint, b: joint.damping * joint.stiffness, c: joint.stiffness, g: b1.composite.torque.z, y_0: 0, y_ddt_0: 0, at: Float(delta))

            XCTAssertEqual(
                float3x3(
                    SIMD3<Float>(0,0,θ[0]),
                    SIMD3<Float>(0,0,θ[1]),
                    SIMD3<Float>(0,0,θ[2])
                ),
                joint.θ, accuracy: 0.0001)
        }

        do {
            let joint = b0.parentJoint!

            let momentOfInertia: Float = b0.composite.inertiaTensor[2,2]
            let compositeInertiaRelativeToJoint = momentOfInertia +
                b0.composite.mass * sqr(distance(b0.composite.centerOfMass, joint.position))

            let θ = evaluateDifferential(a: compositeInertiaRelativeToJoint, b: joint.damping * joint.stiffness, c: joint.stiffness, g: b0.composite.torque.z, y_0: 0, y_ddt_0: 0, at: Float(delta))

            XCTAssertEqual(
                float3x3(
                    SIMD3<Float>(0,0,θ[0]),
                    SIMD3<Float>(0,0,θ[1]),
                    SIMD3<Float>(0,0,θ[2])
                ),
                joint.θ, accuracy: 0.0001)
        }
    }

    func testUpdateRigidBodies() {
        let jointPosition_b0_beforeUpdate = b0.parentJoint!.position
        let jointPosition_b1_beforeUpdate = b1.parentJoint!.position

        simulator.update(at: delta)

        do {
            let joint = b0.parentJoint!
            let momentOfInertia: Float = b0.composite.inertiaTensor[2,2]
            let compositeInertiaRelativeToJoint = momentOfInertia +
                b0.composite.mass * sqr(distance(b0.composite.centerOfMass, jointPosition_b0_beforeUpdate))
            let θ = evaluateDifferential(a: compositeInertiaRelativeToJoint, b: joint.damping * joint.stiffness, c: joint.stiffness, g: b0.composite.torque.z, y_0: 0, y_ddt_0: 0, at: Float(delta))

            let rotation = simd_quatf(angle: θ[0], axis: .z)
            XCTAssertEqual(
                b0.pivot + rotation.act(SIMD3<Float>(0, 0.5, 0)),
                b0.centerOfMass, accuracy: 0.0001)
            XCTAssertEqual(
                rotation,
                b0.rotation, accuracy: 0.0001)
        }

        do {
            let joint = b1.parentJoint!
            let momentOfInertia: Float = 1/4 + 1/12
            let compositeInertiaRelativeToJoint = momentOfInertia +
                b1.composite.mass * sqr(distance(b1.composite.centerOfMass, jointPosition_b1_beforeUpdate))

            let θ = evaluateDifferential(a: compositeInertiaRelativeToJoint, b: joint.damping * joint.stiffness, c: joint.stiffness, g: b1.composite.torque.z, y_0: 0, y_ddt_0: 0, at: Float(delta))

            let rotation = simd_quatf(angle: θ[0], axis: .z)
            XCTAssertEqual(
                b1.pivot + (joint.rotation * rotation).normalized.act(SIMD3<Float>(0, 0.5, 0)),
                b1.centerOfMass, accuracy: 0.0001)
            XCTAssertEqual(
                (joint.rotation * rotation).normalized,
                b1.rotation, accuracy: 0.0001)
        }
    }

}
