import Foundation
import XCTest
@testable import TreePhysics
import MetalKit
import ShaderTypes

fileprivate let sqrt2: Float = sqrt(2)
fileprivate let delta: TimeInterval = 1.0/60

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
    var updateJoints: UpdateJoints!
    var updateRigidBodies: UpdateRigidBodies!

    var device: MTLDevice!, commandQueue: MTLCommandQueue!
    var mem: MemoryLayoutManager!

    var b0: ArticulatedRigidBody!
    var b1: ArticulatedRigidBody!
    let force = simd_float3(1, 0, 0) // world coordinates
    var forceAppliedPosition: simd_float3!
    let momentOfInertiaOfRod: Float = 1/4 + 1/12

    override func setUp() {
        super.setUp()

        self.device = SharedBuffersMTLDevice(MTLCreateSystemDefaultDevice()!)
        self.commandQueue = device.makeCommandQueue()!

        let root = ArticulatedRigidBody.static()
        b0 = Tree.internode(length: 1, radius: 1)
        b1 = Tree.internode(length: 1, radius: 1)
        let b0joint = root.add(b0, orientation: .identity, position: .zero)
        b0joint.stiffness = 1
        b0joint.torqueThreshold = .infinity
        b0joint.damping = 1

        let b1Joint = b0.add(b1, orientation: simd_quatf(angle: -.pi/4, axis: .z), position: simd_float3(0,1,0))
        b1Joint.stiffness = 1
        b1Joint.torqueThreshold = .infinity
        b1Joint.damping = 1

        b1.apply(force: force)

        self.mem = MemoryLayoutManager(device: device, root: root, fields: [])
        self.updateCompositeBodies = UpdateCompositeBodies(device: device, memoryLayoutManager: mem)
        self.updateJoints = UpdateJoints(device: device, memoryLayoutManager: mem)
        self.updateRigidBodies = UpdateRigidBodies(device: device, memoryLayoutManager: mem)
    }

    func testUpdateCompositeBodies() {
        let expect = expectation(description: "wait")

        let commandBuffer = commandQueue.makeCommandBuffer()!
        updateCompositeBodies.encode(commandBuffer: commandBuffer)
        commandBuffer.addCompletedHandler { _ in
            let b1_θ = self.mem.joints[self.mem.rigidBodies.index[self.b1]!]
            let b0_θ = self.mem.joints[self.mem.rigidBodies.index[self.b0]!]

            // torque
            XCTAssertEqual(b1_θ.torque, self.b1.parentJoint!.rotate(vector: self.b1.torque), accuracy: 0.0001)
            XCTAssertEqual(b0_θ.torque, self.b0.parentJoint!.rotate(vector: self.b1.torque + cross(self.b1.pivot - self.b0.pivot, self.b1.force)), accuracy: 0.0001)

            // inertia tensor
            let b1_θ_inertiaTensor = self.b1.parentJoint!.rotate(tensor: self.b1.inertiaTensor) - self.b1.mass * sqr(
                self.b1.parentJoint!.rotate(vector: self.b1.centerOfMass - self.b1.pivot).skew)
            XCTAssertEqual(b1_θ.inertiaTensor, b1_θ_inertiaTensor, accuracy: 0.0001)

            let b0_θ_mass = self.b1.mass + self.b0.mass
            let b0_θ_centerOfMass = (self.b1.centerOfMass + self.b0.centerOfMass) / 2
            var b0_θ_inertiaTensor = self.b0.inertiaTensor - self.b0.mass * sqr((self.b0.centerOfMass - b0_θ_centerOfMass).skew)
            b0_θ_inertiaTensor += self.b1.inertiaTensor - self.b1.mass * sqr((self.b1.centerOfMass - b0_θ_centerOfMass).skew)
            b0_θ_inertiaTensor = self.b0.parentJoint!.rotate(tensor: b0_θ_inertiaTensor) - b0_θ_mass * sqr(self.b0.parentJoint!.rotate(vector: b0_θ_centerOfMass - self.b0.pivot).skew)
            XCTAssertEqual(b0_θ.inertiaTensor, b0_θ_inertiaTensor, accuracy: 0.001)

            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }

    func testUpdateJoints() {
        let expect = expectation(description: "wait")

        let commandBuffer = commandQueue.makeCommandBuffer()!
        updateCompositeBodies.encode(commandBuffer: commandBuffer)
        updateJoints.encode(commandBuffer: commandBuffer, at: 1/60)
        commandBuffer.addCompletedHandler { _ in
            let b1_θ = self.mem.joints[self.mem.rigidBodies.index[self.b1]!]
            let b0_θ = self.mem.joints[self.mem.rigidBodies.index[self.b0]!]

            // thetas
            do {
                let joint = self.b1.parentJoint!
                let momentOfInertia: Float = self.momentOfInertiaOfRod
                let compositeInertiaRelativeToJoint = momentOfInertia +
                    self.b1.mass * sqr(distance(self.b1.centerOfMass, self.b1.pivot))
                let θ = evaluateDifferential(a: compositeInertiaRelativeToJoint, b: joint.damping * joint.stiffness, c: joint.stiffness, g: self.b1.torque.z, y_0: 0, y_ddt_0: 0, at: Float(delta))

                XCTAssertEqual(
                    float3x3(
                        simd_float3(0,0,θ[0]),
                        simd_float3(0,0,θ[1]),
                        simd_float3(0,0,θ[2])
                    ),
                    b1_θ.theta, accuracy: 0.0001)
            }

            do {
                let joint = self.b0.parentJoint!
                var centerOfMass: simd_float3 = (self.b0.mass * self.b0.centerOfMass + self.b1.mass * self.b1.centerOfMass)
                centerOfMass /= (self.b0.mass + self.b1.mass)
                var momentOfInertia: Float = 0
                momentOfInertia += self.momentOfInertiaOfRod + self.b1.mass * distance_squared(self.b1.centerOfMass, centerOfMass)
                momentOfInertia += self.momentOfInertiaOfRod + self.b0.mass * distance_squared(self.b0.centerOfMass, centerOfMass)
                let compositeInertiaRelativeToJoint = momentOfInertia +
                    (self.b0.mass + self.b1.mass) * sqr(distance(centerOfMass, joint.position))
                let torque = cross(self.b1.centerOfMass - self.b0.pivot, self.force)
                let θ = evaluateDifferential(a: compositeInertiaRelativeToJoint, b: joint.damping * joint.stiffness, c: joint.stiffness, g: torque.z, y_0: 0, y_ddt_0: 0, at: Float(delta))

                XCTAssertEqual(
                    float3x3(
                        simd_float3(0,0,θ[0]),
                        simd_float3(0,0,θ[1]),
                        simd_float3(0,0,θ[2])
                    ),
                    b0_θ.theta, accuracy: 0.0001)
            }

            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }

    func testUpdateRigidBodies() {
        let expect = expectation(description: "wait")

        let commandBuffer = commandQueue.makeCommandBuffer()!
        updateCompositeBodies.encode(commandBuffer: commandBuffer)
        updateJoints.encode(commandBuffer: commandBuffer, at: 1/60)
        updateRigidBodies.encode(commandBuffer: commandBuffer)
        commandBuffer.addCompletedHandler { _ in
            do {
                let joint = self.b0.parentJoint!
                var centerOfMass: simd_float3 = (self.b0.mass * self.b0.centerOfMass + self.b1.mass * self.b1.centerOfMass)
                centerOfMass /= (self.b0.mass + self.b1.mass)
                var momentOfInertia: Float = 0
                momentOfInertia += self.momentOfInertiaOfRod + self.b1.mass * distance_squared(self.b1.centerOfMass, centerOfMass)
                momentOfInertia += self.momentOfInertiaOfRod + self.b0.mass * distance_squared(self.b0.centerOfMass, centerOfMass)
                let compositeInertiaRelativeToJoint = momentOfInertia +
                    (self.b0.mass + self.b1.mass) * sqr(distance(centerOfMass, joint.position))
                let torque = cross(self.b1.centerOfMass - self.b0.pivot, self.force)
                let θ = evaluateDifferential(a: compositeInertiaRelativeToJoint, b: joint.damping * joint.stiffness, c: joint.stiffness, g: torque.z, y_0: 0, y_ddt_0: 0, at: Float(delta))

                let orientation = simd_quatf(angle: θ[0], axis: .z)
                XCTAssertEqual(
                    self.b0.pivot + orientation.act(simd_float3(0, 0.5, 0)),
                    self.b0.centerOfMass, accuracy: 0.0001)
                XCTAssertEqual(
                    orientation,
                    self.mem.rigidBodies[self.b0].orientation, accuracy: 0.0001)
            }

            do {
                let joint = self.b1.parentJoint!
                let momentOfInertia: Float = self.momentOfInertiaOfRod
                let compositeInertiaRelativeToJoint = momentOfInertia +
                    self.b1.mass * sqr(distance(self.b1.centerOfMass, self.b1.pivot))
                let θ = evaluateDifferential(a: compositeInertiaRelativeToJoint, b: joint.damping * joint.stiffness, c: joint.stiffness, g: self.b1.torque.z, y_0: 0, y_ddt_0: 0, at: Float(delta))

                let orientation = simd_quatf(angle: θ[0], axis: .z)
                XCTAssertEqual(
                    self.b1.pivot + (joint.orientation * orientation).normalized.act(simd_float3(0, 0.5, 0)),
                    self.b1.centerOfMass, accuracy: 0.0001)
                XCTAssertEqual(
                    (joint.orientation * orientation).normalized,
                    self.mem.rigidBodies[self.b1].orientation, accuracy: 0.001)
            }

            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }
}
