import XCTest
@testable import TreePhysics
import MetalKit

class TreePhysicsTests: XCTestCase {
    func testComposite() {
        let root = Branch()
        let b1 = Branch()
        let b2 = Branch()
        root.add(b1)
        b1.add(b2)

        let force = float2(0, 1) // world coordinates
        let r_local = float2(0, 1)
        let r_world = b2.convert(position: r_local)
        b2.apply(force: force, at: 1) // ie at float2(0, 1) in local coordinates
        XCTAssertEqual(b2.mass, 1)
        XCTAssertEqual(b2.force, force)
        let r_b2 = r_world - b2.worldPosition
        XCTAssertEqual(r_b2, float2(1, 0), accuracy: 0.0001)
        XCTAssertEqual(b2.torque, cross(force, r_b2))
        XCTAssertEqual(b2.inertia, 1.0/12 * 1 * 1) // moment of inertia is relative to center of mass
        XCTAssertEqual(b2.worldCenterOfMass, float2(0.5 + 1/sqrt(2), 1 + 1/sqrt(2)), accuracy: 0.0001)

        // position
        XCTAssertEqual(b2.position, float2(0,1))
        XCTAssertEqual(b2.worldPosition, float2(1/sqrt(2), 1 + 1/sqrt(2)))
        XCTAssertEqual(b1.position, float2(0,1))
        XCTAssertEqual(b1.worldPosition, float2(0,1))
        XCTAssertEqual(root.position, float2.zero)

        root.updateComposite()

        // mass
        XCTAssertEqual(b2.compositeMass, 1)
        XCTAssertEqual(b1.compositeMass, 2)
        XCTAssertEqual(root.compositeMass, 3)

        // force
        XCTAssertEqual(b2.compositeForce, force)
        XCTAssertEqual(b1.compositeForce, force)
        XCTAssertEqual(root.compositeForce, force)

        // torque
        XCTAssertEqual(b2.compositeTorque, b2.torque)
        let r_b1 = r_world - b1.worldPosition
        XCTAssertEqual(b1.compositeTorque, cross(force, r_b1))
        let r_root = r_world - root.worldPosition
        XCTAssertEqual(root.compositeTorque, cross(force, r_root))

        // center of mass
        XCTAssertEqual(b2.compositeCenterOfMass, b2.worldCenterOfMass)
        XCTAssertEqual(b1.compositeCenterOfMass, (b1.worldCenterOfMass + b2.worldCenterOfMass)/2)
        XCTAssertEqual(root.compositeCenterOfMass, (b1.worldCenterOfMass + b2.worldCenterOfMass + root.worldCenterOfMass) / 3)

        // inertia
        XCTAssertEqual(b2.compositeInertia, b2.inertia)
        XCTAssertEqual(b1.compositeInertia,
                       b1.inertia + b1.mass * square(distance(b1.worldCenterOfMass, b1.compositeCenterOfMass)) +
                        b2.inertia + b2.mass * square(distance(b2.worldCenterOfMass, b1.compositeCenterOfMass)),
                       accuracy: 0.0001)
        XCTAssertEqual(root.compositeInertia,
                       root.inertia + root.mass * square(distance(root.worldCenterOfMass, root.compositeCenterOfMass)) +
                        b1.inertia + b1.mass * square(distance(b1.worldCenterOfMass, root.compositeCenterOfMass)) +
                        b2.inertia + b2.mass * square(distance(b2.worldCenterOfMass, root.compositeCenterOfMass)),
                       accuracy: 0.0001)
    }
}

func XCTAssertEqual(_ x: float2, _ y: float2, accuracy: Float, file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(x.x, y.x, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(x.y, y.y, accuracy: accuracy, file: file, line: line)
}
