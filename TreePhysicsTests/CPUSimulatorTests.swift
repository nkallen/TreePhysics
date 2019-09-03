import XCTest
@testable import TreePhysics
import simd

fileprivate let sqrt2: Float = sqrtf(2)

class CPUSimulatorTests: XCTestCase {
    var simulator: CPUSimulator!
    var root: RigidBody!
    var b1: RigidBody!
    var b2: RigidBody!
    var forceAppliedPosition: float3!
    let force = float3(0, 1, 0) // world coordinates

    override func setUp() {
        super.setUp()

        root = RigidBody()
        b1 = RigidBody()
        b2 = RigidBody()
        _ = root.add(b1)
        _ = b1.add(b2)

        simulator = CPUSimulator(root: root)

        b2.apply(force: force, at: 1) // ie at float3(0, 1, 0) in local coordinates
        self.forceAppliedPosition = b2.translation + b2.rotation.act(float3(0, 1, 0))
    }

    func testApplyForce() {
        XCTAssertEqual(b2.mass, 1)
        XCTAssertEqual(b2.force, force)
        XCTAssertEqual(b2.length, 1)
        XCTAssertEqual(b2.radius, 1)

        XCTAssertEqual(b2.torque, cross(forceAppliedPosition - b2.position, force))

        XCTAssertEqual(float3x3(b2.rotation.normalized) * b2.inertiaTensor * float3x3(b2.rotation.normalized).transpose,
                       float3x3(diagonal: float3(
                        1.0/4 + 1.0/12,
                        1.0/2,
                        1.0/4 + 1.0/12
                       )), accuracy: 0.0001)

        XCTAssertEqual(root.position, float3.zero)
        XCTAssertEqual(b1.parentJoint!.position, float3(0,1,0))
        XCTAssertEqual(b1.position, float3(0,1,0))

        XCTAssertEqual(b2.centerOfMass, float3(0.5 + 1/sqrt2, 1 + 1/sqrt2, 0), accuracy: 0.0001)
        XCTAssertEqual(b1.centerOfMass, float3(0.5/sqrt2, 1 + 0.5/sqrt2, 0), accuracy: 0.0001)

        XCTAssertEqual(b2.parentJoint!.position, float3(1/sqrt2, 1 + 1/sqrt2, 0), accuracy: 0.0001)
        XCTAssertEqual(b1.parentJoint!.position, float3(0,1, 0))
    }

    func testComposite() {
        simulator.updateCompositeBodies()

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
                float3(0,0,0.00022747826), // i.e., a small rotation about the z axis
                float3(0,0,0.0265728),
                float3(0,0,1.4540794)
            ),
            b2.parentJoint!.θ, accuracy: 0.0001)

        XCTAssertEqual(
            float3x3(
                float3(0,0,8.173804e-5), // a larger torque but also larger inertia
                float3(0,0,0.009755039),
                float3(0,0,0.5747631)
            ),
            b1.parentJoint!.θ, accuracy: 0.0001)
    }

    func testUpdateRigidBodies() {
        XCTAssertEqual(
            float3(1/sqrt2, 1+1/sqrt2, 0),
            b2.position, accuracy: 0.0001)
        XCTAssertEqual(
            simd_quatf(angle: .pi/2, axis: float3(0,0,-1)),
            b2.rotation, accuracy: 0.0001)

        XCTAssertEqual(
            float3(0, 1, 0),
            b1.position)
        XCTAssertEqual(
            simd_quatf(angle: .pi/4, axis: float3(0,0,-1)),
            b1.rotation, accuracy: 0.0001)

        simulator.update(at: 1.0 / 30)

        XCTAssertEqual(
            float3(0.70687836, 1.7073351, 0),
            b2.position, accuracy: 0.0001)
        XCTAssertEqual(
            simd_quatf(angle: 1.5696167, axis: float3(0,0,-1)),
            b2.rotation, accuracy: 0.0001)

        XCTAssertEqual(
            float3(0, 1, 0),
            b1.position)
        XCTAssertEqual(
            simd_quatf(angle: 0.78507507, axis: float3(0,0,-1)),
            b1.rotation, accuracy: 0.0001)
    }

}
