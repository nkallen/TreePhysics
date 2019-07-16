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
    }
}

func XCTAssertEqual(_ x: float2, _ y: float2, accuracy: Float, file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(x.x, y.x, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(x.y, y.y, accuracy: accuracy, file: file, line: line)
}
