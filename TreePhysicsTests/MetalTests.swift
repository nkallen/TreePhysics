import Foundation
import XCTest
@testable import TreePhysics
import MetalKit

fileprivate let sqrt2: Float = sqrt(2)

class UpdateCompositeBodiesKernelTests: XCTestCase {
    var updateCompositeBodiesKernel: UpdateCompositeBodiesKernel!

    var device: MTLDevice!
    var commandQueue: MTLCommandQueue!
    var compositeBodiesBuffer: MTLBuffer!

    var root, b1, b2: RigidBody!
    let force: float3 = float3(0, 1, 0) // world coordinates

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

        let (count, rigidBodiesBuffer, ranges) = UpdateCompositeBodiesKernel.buffer(root: root, device: device)
        self.compositeBodiesBuffer = device.makeBuffer(
            length: MemoryLayout<CompositeBodyStruct>.stride * count,
            options: [.storageModeShared])!

        self.updateCompositeBodiesKernel = UpdateCompositeBodiesKernel(rigidBodiesBuffer: rigidBodiesBuffer, ranges: ranges, compositeBodiesBuffer: compositeBodiesBuffer)
    }

    func testUpdateCompositeBodies() {
        let forceAppliedPosition = b2.convert(position: float3(0, 1, 0))

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
            let r_b1 = forceAppliedPosition - self.b1.parentJoint!.position
            XCTAssertEqual(b1_composite.torque, cross(r_b1, self.force))
            let r_root = forceAppliedPosition - self.root.position
            XCTAssertEqual(root_composite.torque, cross(r_root, self.force))

            // center of mass
            XCTAssertEqual(b2_composite.centerOfMass, self.b2.centerOfMass)
            XCTAssertEqual(b1_composite.centerOfMass, (self.b1.centerOfMass + self.b2.centerOfMass)/2)
            XCTAssertEqual(root_composite.centerOfMass, (self.b1.centerOfMass + self.b2.centerOfMass + self.root.centerOfMass) / 3, accuracy: 0.001)

            // inertia tensor
            XCTAssertEqual(b2_composite.inertiaTensor, self.b2.inertiaTensor)
            var b1_inertiaTensor = self.b1.inertiaTensor - self.b1.mass * sqr((self.b1.centerOfMass - b1_composite.centerOfMass).crossMatrix)
            b1_inertiaTensor += b2_composite.inertiaTensor - b2_composite.mass * sqr((b2_composite.centerOfMass - b1_composite.centerOfMass).crossMatrix)
            XCTAssertEqual(b1_composite.inertiaTensor, b1_inertiaTensor, accuracy: 0.0001)
            var root_inertiaTensor = self.root.inertiaTensor - self.root.mass * sqr((self.root.centerOfMass - root_composite.centerOfMass).crossMatrix)
            root_inertiaTensor += b1_composite.inertiaTensor - b1_composite.mass * sqr((b1_composite.centerOfMass - root_composite.centerOfMass).crossMatrix)
            XCTAssertEqual(root_composite.inertiaTensor, root_inertiaTensor, accuracy: 0.0001)

            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }
}

class UpdateJointsKernelTests: XCTestCase {
    var updateCompositeBodiesKernel: UpdateCompositeBodiesKernel!
    var updateJointsKernel: UpdateJointsKernel!

    var device: MTLDevice!
    var commandQueue: MTLCommandQueue!
    var compositeBodiesBuffer, jointsBuffer: MTLBuffer!

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
        self.jointsBuffer = UpdateJointsKernel.buffer(count: count, device: device)

        self.updateCompositeBodiesKernel = UpdateCompositeBodiesKernel(device: device, rigidBodiesBuffer: rigidBodiesBuffer, ranges: ranges, compositeBodiesBuffer: compositeBodiesBuffer)
        self.updateJointsKernel = UpdateJointsKernel(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, numJoints: count)
    }

    func testUpdateJoints() {
        let expect = expectation(description: "wait")

        let commandBuffer = commandQueue.makeCommandBuffer()!
        updateCompositeBodiesKernel.encode(commandBuffer: commandBuffer)
        updateJointsKernel.encode(commandBuffer: commandBuffer, at: 1/60)
        commandBuffer.addCompletedHandler { _ in
            let joints = UnsafeMutableRawPointer(self.jointsBuffer.contents()).bindMemory(to: JointStruct.self, capacity: 2)

            let b2_parentJoint = joints[0]
            let b1_parentJoint = joints[1]

            XCTAssertEqual(
                float3x3(
                    float3(0,0,0.005), // i.e., a small rotation about the z axis
                    float3(0,0,0),
                    float3(0,0,0)
                ),
                b2_parentJoint.θ, accuracy: 0.0001)

            XCTAssertEqual(
                float3x3(
                    float3(0,0,0.008535533), // a slightly larger rotation since the torque on b1 is greater
                    float3(0,0,0),
                    float3(0,0,0)
                ),
                b1_parentJoint.θ, accuracy: 0.0001)

            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }
}

class UpdateRigidBodiesKernelTests: XCTestCase {
    var updateCompositeBodiesKernel: UpdateCompositeBodiesKernel!
    var updateJointsKernel: UpdateJointsKernel!
    var updateRigidBodiesKernel: UpdateRigidBodiesKernel!

    var device: MTLDevice!
    var commandQueue: MTLCommandQueue!
    var compositeBodiesBuffer, jointsBuffer, rigidBodiesBuffer: MTLBuffer!

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
        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.compositeBodiesBuffer = device.makeBuffer(
            length: MemoryLayout<CompositeBodyStruct>.stride * count,
            options: [.storageModeShared])!
        self.jointsBuffer = UpdateJointsKernel.buffer(count: count, device: device)

        self.updateCompositeBodiesKernel = UpdateCompositeBodiesKernel(device: device, rigidBodiesBuffer: rigidBodiesBuffer, ranges: ranges, compositeBodiesBuffer: compositeBodiesBuffer)
        self.updateJointsKernel = UpdateJointsKernel(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, numJoints: count)
        self.updateRigidBodiesKernel = UpdateRigidBodiesKernel(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, ranges: ranges)
    }

    func testUpdateRigidBodies() {
        let expect = expectation(description: "wait")

        let commandBuffer = commandQueue.makeCommandBuffer()!
        updateCompositeBodiesKernel.encode(commandBuffer: commandBuffer)
        updateJointsKernel.encode(commandBuffer: commandBuffer, at: 1/60)
        updateRigidBodiesKernel.encode(commandBuffer: commandBuffer)
        commandBuffer.addCompletedHandler { _ in
            let rigidBodies = UnsafeMutableRawPointer(self.rigidBodiesBuffer.contents()).bindMemory(to: RigidBodyStruct.self, capacity: 2)

            let b2 = rigidBodies[0]
            let b1 = rigidBodies[1]

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
                float3(0, 1, 0),
                b1.position)
            XCTAssertEqual(
                float3x3(
                    float3(0.7131165,-0.7010456,0),
                    float3(0.7010456,0.7131165,0),
                    float3(0,0,1)),
                b1.rotation, accuracy: 0.0001)

            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }
}

