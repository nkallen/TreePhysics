import Foundation
import XCTest
@testable import TreePhysics
import MetalKit

class UpdateCompositeBodiesKernelTests: XCTestCase {
    var updateCompositeBodiesKernel: UpdateCompositeBodiesKernel!

    var device: MTLDevice!
    var commandQueue: MTLCommandQueue!
    var compositeBodiesBuffer: MTLBuffer!

    var root, b1, b2: RigidBody!
    let force: float3 = float3(0, 1, 0) // world coordinates
    var forceAppliedPosition: float3!

    override func setUp() {
        super.setUp()

        self.device = MTLCreateSystemDefaultDevice()!
        self.commandQueue = device.makeCommandQueue()!

        self.root = RigidBody()
        self.b1 = RigidBody()
        self.b2 = RigidBody()
        root.add(b1, at: float3(0,0,-Float.pi/4))
        b1.add(b2, at: float3(0,0,-Float.pi/4))
        b2.apply(force: force, at: 1) // ie at float3(0, 1,  0) in local coordinates
        self.forceAppliedPosition = b2.convert(position: float3(0, 1, 0))

        let (count, rigidBodiesBuffer, ranges) = UpdateCompositeBodiesKernel.buffer(root: root, device: device)
        self.compositeBodiesBuffer = device.makeBuffer(
            length: MemoryLayout<CompositeBodyStruct>.stride * count,
            options: [.storageModeShared])!

        self.updateCompositeBodiesKernel = UpdateCompositeBodiesKernel(rigidBodiesBuffer: rigidBodiesBuffer, ranges: ranges, compositeBodiesBuffer: compositeBodiesBuffer)
    }

    func testUpdateCompositeBodies() {
        let expect = expectation(description: "wait")

        let commandBuffer = commandQueue.makeCommandBuffer()!
        updateCompositeBodiesKernel.encode(commandBuffer: commandBuffer)
        commandBuffer.addCompletedHandler { _ in
            let compositeBodies = UnsafeMutableRawPointer(self.compositeBodiesBuffer.contents()).bindMemory(to: CompositeBodyStruct.self, capacity: 3)
            let b2_composite = compositeBodies[0]
            let b1_composite = compositeBodies[1]
            let root_composite = compositeBodies[2]

            // mass
            XCTAssertEqual(b2_composite.mass, 1)
            XCTAssertEqual(b1_composite.mass, 2)
            XCTAssertEqual(root_composite.mass, 3)

            // force
            XCTAssertEqual(b2_composite.force, self.force)
            XCTAssertEqual(b1_composite.force, self.force)
            XCTAssertEqual(root_composite.force, self.force)

            // torque
            XCTAssertEqual(b2_composite.torque, self.b2.torque)
            let r_b1 = self.forceAppliedPosition - self.b1.parentJoint!.position
            XCTAssertEqual(b1_composite.torque, cross(r_b1, self.force))
            let r_root = self.forceAppliedPosition - self.root.position
            XCTAssertEqual(root_composite.torque, cross(r_root, self.force))

            // center of mass
            XCTAssertEqual(b2_composite.centerOfMass, self.b2.centerOfMass)
            XCTAssertEqual(b1_composite.centerOfMass, (self.b1.centerOfMass + self.b2.centerOfMass)/2)
            XCTAssertEqual(root_composite.centerOfMass, (self.b1.centerOfMass + self.b2.centerOfMass + self.root.centerOfMass) / 3, accuracy: 0.001)

            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }
}
/*
class UpdateJointsKernelTests: XCTestCase {
    var updateJointsKernel: UpdateCompositeBodiesKernel!

    var device: MTLDevice!
    var commandQueue: MTLCommandQueue!

    var root, b1, b2: RigidBody!
    let force: float3 = float3(0, 1, 0) // world coordinates
    var forceAppliedPosition: float3!

    override func setUp() {
        super.setUp()

        self.device = MTLCreateSystemDefaultDevice()!
        self.commandQueue = device.makeCommandQueue()!

        self.root = RigidBody()
        self.b1 = RigidBody()
        self.b2 = RigidBody()
        root.add(b1, at: float3(0,0,-Float.pi/4))
        b1.add(b2, at: float3(0,0,-Float.pi/4))
        b2.apply(force: force, at: 1) // ie at float3(0, 1,  0) in local coordinates
        self.forceAppliedPosition = b2.convert(position: float3(0, 1, 0))
    }

    func testUpdateJoints() {
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
}
*/
