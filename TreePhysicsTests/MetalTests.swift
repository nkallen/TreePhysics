import Foundation
import XCTest
@testable import TreePhysics
import MetalKit


class MetalTests: XCTestCase {
    var diagonalizeKernel: DiagonalizeKernel!

    func testDiagonalize() {
        self.diagonalizeKernel = DiagonalizeKernel()
        let matrix = float3x3(columns: (
            float3(2,1,0),
            float3(1,2,1),
            float3(0,1,2)))

        let expectation = XCTestExpectation(description: "wait")
        diagonalizeKernel.run(matrix) { buffer in
            let eigenvalues = UnsafeMutableRawPointer(buffer.contents()).bindMemory(to: float3.self, capacity: 1024)
            XCTAssertEqual(
                float3(2 + sqrt(2), 2, 2 - sqrt(2.0)),
                eigenvalues[0], accuracy: 0.0001)
            expectation.fulfill()
        }
    }

    var updateCompositeBodiesKernel: UpdateCompositeBodiesKernel!

    func testUpdateCompositeBodies() {
        let expectation = XCTestExpectation(description: "wait")

        let force = float3(0, 1, 0) // world coordinates
        let root = RigidBody()
        let b1 = RigidBody()
        let b2 = RigidBody()
        root.add(b1, at: float3(0,0,-Float.pi/4))
        b1.add(b2, at: float3(0,0,-Float.pi/4))
        b2.apply(force: force, at: 1) // ie at float3(0, 1,  0) in local coordinates
        let forceAppliedPosition = b2.convert(position: float3(0, 1, 0))

        self.updateCompositeBodiesKernel = UpdateCompositeBodiesKernel(rigidBody: root)
        updateCompositeBodiesKernel.run() { buffer in
            let compositeBodies = UnsafeMutableRawPointer(buffer.contents()).bindMemory(to: CompositeBodyStruct.self, capacity: 3)
            let b2_composite = compositeBodies[0]
            let b1_composite = compositeBodies[1]
            let root_composite = compositeBodies[2]

            // mass
            XCTAssertEqual(b2_composite.mass, 1)
            XCTAssertEqual(b1_composite.mass, 2)
            XCTAssertEqual(root_composite.mass, 3)

            // force
            XCTAssertEqual(b2_composite.force, force)
            XCTAssertEqual(b1_composite.force, force)
            XCTAssertEqual(root_composite.force, force)

            // torque
            XCTAssertEqual(b2_composite.torque, b2.torque)
            let r_b1 = forceAppliedPosition - b1.parentJoint!.position
            XCTAssertEqual(b1_composite.torque, cross(r_b1, force))
            let r_root = forceAppliedPosition - root.position
            XCTAssertEqual(root_composite.torque, cross(r_root, force))

            // center of mass
            XCTAssertEqual(b2_composite.centerOfMass, b2.centerOfMass)
            XCTAssertEqual(b1_composite.centerOfMass, (b1.centerOfMass + b2.centerOfMass)/2)
            XCTAssertEqual(root_composite.centerOfMass, (b1.centerOfMass + b2.centerOfMass + root.centerOfMass) / 3)

            expectation.fulfill()
        }
    }

    //    func testFoo() {
    //
    //
    //        guard let (eigenvalues, eigenvectors) = matrix.eigen_ql else {
    //            XCTFail()
    //            return
    //        }
    //
    //        XCTAssertEqual(
    //            double3(2 - sqrt(2), 2, 2 + sqrt(2.0)),
    //            eigenvalues, accuracy: 0.0001)
    //
    //        XCTAssertEqual(
    //            double3x3(columns: (
    //                -double3(1.0 / 2, -sqrt(2.0) / 2, 1.0 / 2),
    //                double3(1 / sqrt(2.0), 0, -1/sqrt(2.0)),
    //                double3(1.0/2, sqrt(2.0) / 2, 1.0/2)
    //            )),
    //            eigenvectors, accuracy: 0.001)
    //    }

}
