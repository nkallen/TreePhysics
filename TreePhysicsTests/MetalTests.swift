import Foundation
import XCTest
@testable import TreePhysics
import MetalKit
import ShaderTypes

fileprivate let sqrt2: Float = sqrt(2)
fileprivate let delta: TimeInterval = 1/60

/*
class ApplyPhysicsFieldsTests: XCTestCase {
    var applyPhysicsFields: ApplyPhysicsFields!
    var device: MTLDevice!, commandQueue: MTLCommandQueue!
    var compositeBodiesBuffer: MTLBuffer!

    var root, b1, b2: ArticulatedRigidBody!

    override func setUp() {
        super.setUp()

        self.device = MTLCreateSystemDefaultDevice()!
        self.commandQueue = device.makeCommandQueue()!

        self.root = Tree.internode()
        self.b1 = Tree.internode()
        self.b2 = Tree.internode()
        _ = root.add(b1)
        _ = b1.add(b2)

        let (rigidBodies, rigidBodiesBuffer, _) = UpdateCompositeBodies.rigidBodiesBuffer(root: root, device: device)

        self.applyPhysicsFields = ApplyPhysicsFields(device: device, rigidBodiesBuffer: rigidBodiesBuffer, numRigidBodies: rigidBodies.count)
    }

    func testUpdate() {
        let expect = expectation(description: "wait")
        let debug = KernelDebugger(device: device)

        let commandBuffer = commandQueue.makeCommandBuffer()!
        applyPhysicsFields.encode(commandBuffer: commandBuffer, field: AttractorField())
        commandBuffer.addCompletedHandler { _ in
            debug.print()
            
            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})

    }
}
 */

class UpdateCompositeBodiesTests: XCTestCase {
    var updateCompositeBodies: UpdateCompositeBodies!

    var device: MTLDevice!, commandQueue: MTLCommandQueue!
    var mem: MemoryLayoutManager!

    var b0: ArticulatedRigidBody!
    var b1: ArticulatedRigidBody!
    let force = simd_float3(1, 0, 0) // world coordinates
    var forceAppliedPosition: simd_float3!

    override func setUp() {
        super.setUp()

        self.device = SharedBuffersMTLDevice(MTLCreateSystemDefaultDevice()!)
        self.commandQueue = device.makeCommandQueue()!

        let root = ArticulatedRigidBody.static()
        b0 = Tree.internode(length: 1, radius: 1)
        b1 = Tree.internode(length: 1, radius: 1)
        let b0joint = root.add(b0, rotation: .identity, position: .zero)
        b0joint.stiffness = 1
        b0joint.torqueThreshold = .infinity
        b0joint.damping = 1

        let b1Joint = b0.add(b1, rotation: simd_quatf(angle: -.pi/4, axis: .z), position: simd_float3(0,1,0))
        b1Joint.stiffness = 1
        b1Joint.torqueThreshold = .infinity
        b1Joint.damping = 1

        b1.apply(force: force)

        self.mem = MemoryLayoutManager(device: device, root: root)
        self.updateCompositeBodies = UpdateCompositeBodies(device: device, memoryLayoutManager: mem)
    }

    func testUpdateCompositeBodiesChecksum() {
        let expect = expectation(description: "wait")

        let commandBuffer = commandQueue.makeCommandBuffer()!
        updateCompositeBodies.encode(commandBuffer: commandBuffer)
        commandBuffer.addCompletedHandler { _ in
            let root_composite = self.mem.compositeBodies[0]!

            // mass
            XCTAssertEqual(root_composite.mass, self.b0.mass + self.b1.mass)

            // force
            XCTAssertEqual(simd_float3(root_composite.force), self.force)

            // torque
            let r_b1 = self.b1.centerOfMass - self.b0.pivot
            XCTAssertEqual(simd_float3(root_composite.torque), cross(r_b1, self.force), accuracy: 0.0001)

            // center of mass
            XCTAssertEqual(simd_float3(root_composite.centerOfMass), (self.b0.centerOfMass + self.b1.centerOfMass)/2, accuracy: 0.0001)

            // inertia tensor
            var b1_inertiaTensor = self.b0.inertiaTensor - self.b0.mass * sqr((self.b0.centerOfMass - simd_float3(root_composite.centerOfMass)).skew)
            b1_inertiaTensor += self.b1.inertiaTensor - self.b1.mass * sqr((self.b1.centerOfMass - root_composite.centerOfMass).skew)
            XCTAssertEqual(root_composite.inertiaTensor, b1_inertiaTensor, accuracy: 0.001)

            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }

    func testUpdateCompositeBodies() {
        let expect = expectation(description: "wait")

        let commandBuffer = commandQueue.makeCommandBuffer()!
        updateCompositeBodies.encode(commandBuffer: commandBuffer)
        commandBuffer.addCompletedHandler { _ in
            let b1_θ = self.mem.joints[self.mem.rigidBodies.index[self.b1]!]
            let b0_θ = self.mem.joints[self.mem.rigidBodies.index[self.b0]!]

            // thetas
            let momentOfInertiaOfRod: Float = 1/4 + 1/12
            do {
                let joint = self.b1.parentJoint!

                let momentOfInertia: Float = momentOfInertiaOfRod
                let compositeInertiaRelativeToJoint = momentOfInertia +
                    self.b1.mass * sqr(distance(self.b1.centerOfMass, self.b1.pivot))

                let θ = evaluateDifferential(a: compositeInertiaRelativeToJoint, b: joint.damping * joint.stiffness, c: joint.stiffness, g: self.b1.torque.z, y_0: 0, y_ddt_0: 0, at: Float(delta))

                XCTAssertEqual(
                    float3x3(
                        simd_float3(0,0,θ[0]),
                        simd_float3(0,0,θ[1]),
                        simd_float3(0,0,θ[2])
                    ),
                    b1_θ, accuracy: 0.0001)
            }

            do {
                let joint = self.b0.parentJoint!

                var centerOfMass: simd_float3 = (self.b0.mass * self.b0.centerOfMass + self.b1.mass * self.b1.centerOfMass)
                centerOfMass /= (self.b0.mass + self.b1.mass)
                var momentOfInertia: Float = 0
                momentOfInertia += momentOfInertiaOfRod + self.b1.mass * distance_squared(self.b1.centerOfMass, centerOfMass)
                momentOfInertia += momentOfInertiaOfRod + self.b0.mass * distance_squared(self.b0.centerOfMass, centerOfMass)
                let compositeInertiaRelativeToJoint = momentOfInertia +
                    (self.b0.mass + self.b1.mass) * sqr(distance(centerOfMass, joint.position))
                let torque = cross(self.b1.centerOfMass - self.b0.pivot, self.force)

                let θ = evaluateDifferential(a: compositeInertiaRelativeToJoint, b: joint.damping * joint.stiffness, c: joint.stiffness, g: torque.z, y_0: 0, y_ddt_0: 0, at: Float(delta))

                XCTAssertEqual(
                    float3x3(
                        SIMD3<Float>(0,0,θ[0]),
                        SIMD3<Float>(0,0,θ[1]),
                        SIMD3<Float>(0,0,θ[2])
                    ),
                    b0_θ, accuracy: 0.0001)
            }

            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }
}
/*

class UpdateRigidBodiesTests: XCTestCase {
    var updateCompositeBodies: UpdateCompositeBodies!
    var updateJoints: UpdateJoints!
    var updateRigidBodies: UpdateRigidBodies!

    var device: MTLDevice!, commandQueue: MTLCommandQueue!
    var compositeBodiesBuffer, jointsBuffer, rigidBodiesBuffer: MTLBuffer!

    var root, b1, b2: ArticulatedRigidBody!
    let force: simd_float3 = simd_float3(0, 1, 0) // world coordinates
    var forceAppliedPosition: float3!

    override func setUp() {
        super.setUp()

        self.device = MTLCreateSystemDefaultDevice()!
        self.commandQueue = device.makeCommandQueue()!

        self.root = Tree.internode()
        self.b1 = Tree.internode()
        self.b2 = Tree.internode()
        _ = root.add(b1)
        _ = b1.add(b2)
        b2.apply(force: force)
        self.forceAppliedPosition = b2.pivot + b2.rotation.act(simd_float3(0, 1, 0))

        let (rigidBodies, rigidBodiesBuffer, ranges) = UpdateCompositeBodies.rigidBodiesBuffer(root: root, device: device)
        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.compositeBodiesBuffer = UpdateCompositeBodies.compositeBodiesBuffer(count: rigidBodies.count, device: device)
        self.jointsBuffer = UpdateJoints.buffer(count: rigidBodies.count, device: device)

        self.updateCompositeBodies = UpdateCompositeBodies(device: device, rigidBodiesBuffer: rigidBodiesBuffer, ranges: ranges, compositeBodiesBuffer: compositeBodiesBuffer)
        self.updateJoints = UpdateJoints(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, numJoints: rigidBodies.count)
        self.updateRigidBodies = UpdateRigidBodies(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, ranges: ranges)
    }

    func testUpdateRigidBodies() {
        let expect = expectation(description: "wait")

        let commandBuffer = commandQueue.makeCommandBuffer()!
        let debug = KernelDebugger(device: device)
        updateCompositeBodies.encode(commandBuffer: debug.wrap(commandBuffer))
        updateJoints.encode(commandBuffer: commandBuffer, at: 1/30)
        updateRigidBodies.encode(commandBuffer: commandBuffer)
        commandBuffer.addCompletedHandler { _ in
            debug.print()
            
            let rigidBodies = UnsafeMutableRawPointer(self.rigidBodiesBuffer.contents()).bindMemory(to: RigidBodyStruct.self, capacity: 2)

            let b2 = rigidBodies[0]
            let b1 = rigidBodies[1]

            XCTAssertEqual(
                simd_float3(0.70687836, 1.7073351, 0),
                simd_float3(b2.pivot), accuracy: 0.001)
            XCTAssertEqual(
                float3x3(
                    simd_float3(0.0011795461,-1,0),
                    simd_float3(1,0.0011795461,0),
                    simd_float3(0,0,1)),
                b2.rotation, accuracy: 0.001)

            XCTAssertEqual(
                simd_float3(0, 1, 0),
                simd_float3(b1.pivot))
            XCTAssertEqual(
                float3x3(
                    simd_float3(0.7073351,-0.70687836,0),
                    simd_float3(0.70687836,0.7073351,0),
                    simd_float3(0,0,1)),
                b1.rotation, accuracy: 0.001)

            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }
}
*/
