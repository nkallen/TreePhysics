import Foundation
import XCTest
@testable import TreePhysics
import MetalKit

fileprivate let sqrt2: Float = sqrt(2)

class ApplyPhysicsFieldsTests: XCTestCase {
    var applyPhysicsFields: ApplyPhysicsFields!
    var device: MTLDevice!, commandQueue: MTLCommandQueue!
    var compositeBodiesBuffer: MTLBuffer!

    var root, b1, b2: RigidBody!

    override func setUp() {
        super.setUp()

        self.device = MTLCreateSystemDefaultDevice()!
        self.commandQueue = device.makeCommandQueue()!

        self.root = RigidBody()
        self.b1 = RigidBody()
        self.b2 = RigidBody()
        root.add(b1, at: float3(0,0,-Float.pi/4))
        b1.add(b2, at: float3(0,0,-Float.pi/4))

        let (rigidBodies, rigidBodiesBuffer, _) = UpdateCompositeBodies.buffer(root: root, device: device)

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

class UpdateCompositeBodiesTests: XCTestCase {
    var updateCompositeBodies: UpdateCompositeBodies!

    var device: MTLDevice!, commandQueue: MTLCommandQueue!
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

        let (rigidBodies, rigidBodiesBuffer, ranges) = UpdateCompositeBodies.buffer(root: root, device: device)
        self.compositeBodiesBuffer = device.makeBuffer(
            length: MemoryLayout<CompositeBodyStruct>.stride * rigidBodies.count,
            options: [.storageModeShared])!

        self.updateCompositeBodies = UpdateCompositeBodies(rigidBodiesBuffer: rigidBodiesBuffer, ranges: ranges, compositeBodiesBuffer: compositeBodiesBuffer)
    }

    func testUpdateCompositeBodies() {
        let forceAppliedPosition = b2.convert(position: float3(0, 1, 0))

        let expect = expectation(description: "wait")
        let debug = KernelDebugger(device: device)

        let commandBuffer = commandQueue.makeCommandBuffer()!
        updateCompositeBodies.encode(commandBuffer: debug.wrap(commandBuffer))
        commandBuffer.addCompletedHandler { _ in
            let compositeBodies = UnsafeMutableRawPointer(self.compositeBodiesBuffer.contents()).bindMemory(to: CompositeBodyStruct.self, capacity: 3)
            let b2_composite = compositeBodies[0]
            let b1_composite = compositeBodies[1]

            // mass
            XCTAssertEqual(b2_composite.mass, 1)
            XCTAssertEqual(b1_composite.mass, 2)

            // force
            XCTAssertEqual(float3(b2_composite.force), self.force)
            XCTAssertEqual(float3(b1_composite.force), self.force)

            // torque
            XCTAssertEqual(float3(b2_composite.torque), self.b2.torque)
            let r_b1 = forceAppliedPosition - self.b1.parentJoint!.position
            XCTAssertEqual(float3(b1_composite.torque), cross(r_b1, self.force), accuracy: 0.0001)

            // center of mass
            XCTAssertEqual(float3(b2_composite.centerOfMass), self.b2.centerOfMass, accuracy: 0.0001)
            XCTAssertEqual(float3(b1_composite.centerOfMass), (self.b1.centerOfMass + self.b2.centerOfMass)/2, accuracy: 0.0001)

            // inertia tensor
            XCTAssertEqual(b2_composite.inertiaTensor, self.b2.inertiaTensor, accuracy: 0.0001)
            var b1_inertiaTensor = self.b1.inertiaTensor - self.b1.mass * sqr((self.b1.centerOfMass - float3(b1_composite.centerOfMass)).crossMatrix)
            b1_inertiaTensor += b2_composite.inertiaTensor - float3x3(b2_composite.mass * sqr((b2_composite.centerOfMass - b1_composite.centerOfMass).crossMatrix))
            XCTAssertEqual(b1_composite.inertiaTensor, b1_inertiaTensor, accuracy: 0.001)

            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }
}

class UpdateJointsTests: XCTestCase {
    var updateCompositeBodies: UpdateCompositeBodies!
    var updateJoints: UpdateJoints!

    var device: MTLDevice!, commandQueue: MTLCommandQueue!
    var compositeBodiesBuffer, jointsBuffer: MTLBuffer!

    var root, b1, b2: RigidBody!
    let force: float3 = float3(0, 1, 0) // world coordinates
    var forceAppliedPosition: float3!

    override func setUp() {
        super.setUp()

        self.device = SharedBuffersMTLDevice(MTLCreateSystemDefaultDevice()!)
        self.commandQueue = device.makeCommandQueue()!

        self.root = RigidBody()
        self.b1 = RigidBody()
        self.b2 = RigidBody()
        root.add(b1, at: float3(0,0,-Float.pi/4))
        b1.add(b2, at: float3(0,0,-Float.pi/4))
        b2.apply(force: force, at: 1) // ie at float3(0, 1,  0) in local coordinates
        self.forceAppliedPosition = b2.convert(position: float3(0, 1, 0))

        let (rigidBodies, rigidBodiesBuffer, ranges) = UpdateCompositeBodies.buffer(root: root, device: device)
        self.compositeBodiesBuffer = device.makeBuffer(
            length: MemoryLayout<CompositeBodyStruct>.stride * rigidBodies.count,
            options: [.storageModeShared])!
        self.jointsBuffer = UpdateJoints.buffer(count: rigidBodies.count, device: device)

        self.updateCompositeBodies = UpdateCompositeBodies(device: device, rigidBodiesBuffer: rigidBodiesBuffer, ranges: ranges, compositeBodiesBuffer: compositeBodiesBuffer)
        self.updateJoints = UpdateJoints(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, numJoints: rigidBodies.count)
    }

    func testUpdateJoints() {
        let expect = expectation(description: "wait")

        let commandBuffer = commandQueue.makeCommandBuffer()!
        updateCompositeBodies.encode(commandBuffer: commandBuffer)
        updateJoints.encode(commandBuffer: commandBuffer, at: 1/60)
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
                float3x3(b2_parentJoint.θ), accuracy: 0.0001)

            XCTAssertEqual(
                float3x3(
                    float3(0,0,0.008535533), // a slightly larger rotation since the torque on b1 is greater
                    float3(0,0,0),
                    float3(0,0,0)
                ),
                float3x3(b1_parentJoint.θ), accuracy: 0.0001)

            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }
}

class UpdateRigidBodiesTests: XCTestCase {
    var updateCompositeBodies: UpdateCompositeBodies!
    var updateJoints: UpdateJoints!
    var updateRigidBodies: UpdateRigidBodies!

    var device: MTLDevice!, commandQueue: MTLCommandQueue!
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

        let (rigidBodies, rigidBodiesBuffer, ranges) = UpdateCompositeBodies.buffer(root: root, device: device)
        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.compositeBodiesBuffer = device.makeBuffer(
            length: MemoryLayout<CompositeBodyStruct>.stride * rigidBodies.count,
            options: [.storageModeShared])!
        self.jointsBuffer = UpdateJoints.buffer(count: rigidBodies.count, device: device)

        self.updateCompositeBodies = UpdateCompositeBodies(device: device, rigidBodiesBuffer: rigidBodiesBuffer, ranges: ranges, compositeBodiesBuffer: compositeBodiesBuffer)
        self.updateJoints = UpdateJoints(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, numJoints: rigidBodies.count)
        self.updateRigidBodies = UpdateRigidBodies(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, ranges: ranges)
    }

    func testUpdateRigidBodies() {
        let expect = expectation(description: "wait")

        let commandBuffer = commandQueue.makeCommandBuffer()!
        let debug = KernelDebugger(device: device)
        updateCompositeBodies.encode(commandBuffer: commandBuffer)
        updateJoints.encode(commandBuffer: commandBuffer, at: 1/60)
        updateRigidBodies.encode(commandBuffer: debug.wrap(commandBuffer))
        commandBuffer.addCompletedHandler { _ in
            let rigidBodies = UnsafeMutableRawPointer(self.rigidBodiesBuffer.contents()).bindMemory(to: RigidBodyStruct.self, capacity: 2)

            let b2 = rigidBodies[0]
            let b1 = rigidBodies[1]

            XCTAssertEqual(
                float3(0.7010456, 1.7131165, 0),
                float3(b2.position), accuracy: 0.001)
            XCTAssertEqual(
                float3x3(
                    float3(0.013535142,-1,0),
                    float3(1,0.013535142,0),
                    float3(0,0,1)),
                float3x3(b2.rotation), accuracy: 0.001)

            XCTAssertEqual(
                float3(0, 1, 0),
                float3(b1.position))
            XCTAssertEqual(
                float3x3(
                    float3(0.7131165,-0.7010456,0),
                    float3(0.7010456,0.7131165,0),
                    float3(0,0,1)),
                float3x3(b1.rotation), accuracy: 0.001)

            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }
}

// FIXME: make both of these use the metal simulator.
class AdvancedMetalTests: XCTestCase {
    var updateCompositeBodies: UpdateCompositeBodies!
    var updateJoints: UpdateJoints!
    var updateRigidBodies: UpdateRigidBodies!
    var resetForces: ResetForces!

    var device: MTLDevice!
    var commandQueue: MTLCommandQueue!
    var compositeBodiesBuffer, jointsBuffer, rigidBodiesBuffer: MTLBuffer!

    var root, b1, b2, b3, b4, b5, b6, b7, b8, b9: RigidBody!
    let force: float3 = float3(0, 1, 0) // world coordinates

    var cpuSimulator: CPUSimulator!

    var expecteds: [RigidBody]!

    override func setUp() {
        super.setUp()

        self.device = SharedBuffersMTLDevice(MTLCreateSystemDefaultDevice()!)
        self.commandQueue = device.makeCommandQueue()!

        self.root = RigidBody()
        self.b1 = RigidBody()
        self.b2 = RigidBody()
        self.b3 = RigidBody()
        self.b4 = RigidBody()
        self.b5 = RigidBody()
        self.b6 = RigidBody()
        self.b7 = RigidBody()
        self.b8 = RigidBody()
        self.b9 = RigidBody()

        root.add(b1)
        b1.add(b2)
        b1.add(b3)
        b3.add(b4)
        b4.add(b5)
        b5.add(b6)
        b5.add(b7)
        b7.add(b8)
        b7.add(b9)

        let (rigidBodies, rigidBodiesBuffer, ranges) = UpdateCompositeBodies.buffer(root: root, device: device)
        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.compositeBodiesBuffer = device.makeBuffer(
            length: MemoryLayout<CompositeBodyStruct>.stride * rigidBodies.count,
            options: [.storageModeShared])!
        self.jointsBuffer = UpdateJoints.buffer(count: rigidBodies.count, device: device)

        self.updateCompositeBodies = UpdateCompositeBodies(device: device, rigidBodiesBuffer: rigidBodiesBuffer, ranges: ranges, compositeBodiesBuffer: compositeBodiesBuffer)
        self.updateJoints = UpdateJoints(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, numJoints: rigidBodies.count)
        self.updateRigidBodies = UpdateRigidBodies(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, ranges: ranges)
        self.resetForces = ResetForces(device: device, rigidBodiesBuffer: rigidBodiesBuffer, numRigidBodies: rigidBodies.count)

        cpuSimulator = CPUSimulator(tree: Tree(root))
        self.expecteds = rigidBodies
    }

    func testUpdate() {
        let expect = expectation(description: "wait")
        let debug = KernelDebugger(device: device)

        let commandBuffer = commandQueue.makeCommandBuffer()!
        updateCompositeBodies.encode(commandBuffer: debug.wrap(commandBuffer))
        updateJoints.encode(commandBuffer: commandBuffer, at: 1.0/60)
        updateRigidBodies.encode(commandBuffer: commandBuffer)
        resetForces.encode(commandBuffer: commandBuffer)

        cpuSimulator.update(at: 1.0 / 60)

        commandBuffer.addCompletedHandler { _ in
            let rigidBodies = UnsafeMutableRawPointer(self.rigidBodiesBuffer.contents()).bindMemory(to: RigidBodyStruct.self, capacity: self.expecteds.count)
            let compositeBodies = UnsafeMutableRawPointer(self.compositeBodiesBuffer.contents()).bindMemory(to: CompositeBodyStruct.self, capacity: self.expecteds.count)
            let joints = UnsafeMutableRawPointer(self.jointsBuffer.contents()).bindMemory(to: JointStruct.self, capacity: self.expecteds.count)

            for i in 0..<(self.expecteds.count-1) {
                let compositeBody = compositeBodies[i]
                let joint = joints[i]
                let rigidBody = rigidBodies[i]
                let expected = self.expecteds[i]

                XCTAssertEqual(expected.composite, compositeBody, accuracy: 0.01)
                if let parentJoint = expected.parentJoint {
                    XCTAssertEqual(parentJoint, joint, accuracy: 0.0001)
                }
                XCTAssertEqual(expected, rigidBody, accuracy: 0.001)
            }

            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }
}

class EvenMoreAdvancedMetalTests: XCTestCase {
    var updateCompositeBodies: UpdateCompositeBodies!
    var updateJoints: UpdateJoints!
    var updateRigidBodies: UpdateRigidBodies!

    var device: MTLDevice!, commandQueue: MTLCommandQueue!
    var compositeBodiesBuffer, jointsBuffer, rigidBodiesBuffer: MTLBuffer!

    var cpuSimulator: CPUSimulator!
    var expecteds: [RigidBody]!

    override func setUp() {
        super.setUp()
        self.device = SharedBuffersMTLDevice(MTLCreateSystemDefaultDevice()!)
        self.commandQueue = device.makeCommandQueue()!

        let root = RigidBody(length: 0, radius: 0, density: 0, kind: .static)
        let rigidBodyPen = RigidBodyPen(parent: root)
        let rule = Rewriter.Rule(symbol: "A", replacement: #"[!"&FA]/////[!"&FA]/////[!"&FA]"#)
        let lSystem = Rewriter.rewrite(premise: "A", rules: [rule], generations: 1)
        let configuration = Interpreter<RigidBodyPen>.Configuration(
            randomScale: 0.4,
            angle: 18 * .pi / 180,
            thickness: 0.002*0.002*Float.pi,
            thicknessScale: 0.9,
            stepSize: 0.1,
            stepSizeScale: 0.9)
        let interpreter = Interpreter(configuration: configuration, pen: rigidBodyPen)
        interpreter.interpret(lSystem)

        let tree = Tree(root)
        self.cpuSimulator = CPUSimulator(tree: tree)

        let (rigidBodies, rigidBodiesBuffer, ranges) = UpdateCompositeBodies.buffer(root: root, device: device)
        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.compositeBodiesBuffer = device.makeBuffer(
            length: MemoryLayout<CompositeBodyStruct>.stride * rigidBodies.count,
            options: [.storageModeShared])!
        self.updateCompositeBodies = UpdateCompositeBodies(device: device, rigidBodiesBuffer: rigidBodiesBuffer, ranges: ranges, compositeBodiesBuffer: compositeBodiesBuffer)

        self.jointsBuffer = UpdateJoints.buffer(count: rigidBodies.count, device: device)
        self.updateJoints = UpdateJoints(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, numJoints: rigidBodies.count)

        self.updateRigidBodies = UpdateRigidBodies(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, ranges: ranges)

        self.expecteds = rigidBodies
    }

    func testUpdate() {
        let expect = expectation(description: "wait")
        let debug = KernelDebugger(device: device)

        let commandBuffer = commandQueue.makeCommandBuffer()!
        updateCompositeBodies.encode(commandBuffer: debug.wrap(commandBuffer))
        updateJoints.encode(commandBuffer: commandBuffer, at: 1.0/60)
        updateRigidBodies.encode(commandBuffer: commandBuffer)

        cpuSimulator.update(at: 1.0 / 60)

        commandBuffer.addCompletedHandler { _ in
            let rigidBodies = UnsafeMutableRawPointer(self.rigidBodiesBuffer.contents()).bindMemory(to: RigidBodyStruct.self, capacity: self.expecteds.count)
            let compositeBodies = UnsafeMutableRawPointer(self.compositeBodiesBuffer.contents()).bindMemory(to: CompositeBodyStruct.self, capacity: self.expecteds.count)
            let joints = UnsafeMutableRawPointer(self.jointsBuffer.contents()).bindMemory(to: JointStruct.self, capacity: self.expecteds.count)

            for i in 0..<(self.expecteds.count-1) {
                let compositeBody = compositeBodies[i]
                let joint = joints[i]
                let rigidBody = rigidBodies[i]
                let expected = self.expecteds[i]

                XCTAssertEqual(expected, rigidBody, accuracy: 0.001)
                XCTAssertEqual(expected.composite, compositeBody, accuracy: 0.0001)
                if let parentJoint = expected.parentJoint {
                    XCTAssertEqual(parentJoint, joint, accuracy: 0.0001)
                }
            }
            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }
}
