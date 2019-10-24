import Foundation
import XCTest
@testable import TreePhysics
import MetalKit
import ShaderTypes

fileprivate let sqrt2: Float = sqrt(2)

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

class UpdateCompositeBodiesTests: XCTestCase {
    var updateCompositeBodies: UpdateCompositeBodies!

    var device: MTLDevice!, commandQueue: MTLCommandQueue!
    var compositeBodiesBuffer: MTLBuffer!

    var root, b1, b2: ArticulatedRigidBody!
    let force: SIMD3<Float> = SIMD3<Float>(0, 1, 0) // world coordinates

    override func setUp() {
        super.setUp()

        self.device = SharedBuffersMTLDevice(MTLCreateSystemDefaultDevice()!)
        self.commandQueue = device.makeCommandQueue()!

        self.root = Tree.internode()
        self.b1 = Tree.internode()
        self.b2 = Tree.internode()
        _ = root.add(b1)
        _ = b1.add(b2)
        b2.apply(force: force)

        let (rigidBodies, rigidBodiesBuffer, ranges) = UpdateCompositeBodies.rigidBodiesBuffer(root: root, device: device)
        self.compositeBodiesBuffer = UpdateCompositeBodies.compositeBodiesBuffer(count: rigidBodies.count, device: device)

        self.updateCompositeBodies = UpdateCompositeBodies(rigidBodiesBuffer: rigidBodiesBuffer, ranges: ranges, compositeBodiesBuffer: compositeBodiesBuffer)
    }

    func testUpdateCompositeBodies() {
        let forceAppliedPosition = b2.pivot + b2.rotation.act(SIMD3<Float>(0, 1, 0))

        let expect = expectation(description: "wait")

        let commandBuffer = commandQueue.makeCommandBuffer()!
        updateCompositeBodies.encode(commandBuffer: commandBuffer)
        commandBuffer.addCompletedHandler { _ in
            let compositeBodies = UnsafeMutableRawPointer(self.compositeBodiesBuffer.contents()).bindMemory(to: CompositeBodyStruct.self, capacity: 3)
            let b2_composite = compositeBodies[0]
            let b1_composite = compositeBodies[1]

            // mass
            XCTAssertEqual(b2_composite.mass, 1)
            XCTAssertEqual(b1_composite.mass, 2)

            // force
            XCTAssertEqual(SIMD3<Float>(b2_composite.force), self.force)
            XCTAssertEqual(SIMD3<Float>(b1_composite.force), self.force)

            // torque
            XCTAssertEqual(SIMD3<Float>(b2_composite.torque), self.b2.torque)
            let r_b1 = forceAppliedPosition - self.b1.parentJoint!.position
            XCTAssertEqual(SIMD3<Float>(b1_composite.torque), cross(r_b1, self.force), accuracy: 0.0001)

            // center of mass
            XCTAssertEqual(SIMD3<Float>(b2_composite.centerOfMass), self.b2.centerOfMass, accuracy: 0.0001)
            XCTAssertEqual(SIMD3<Float>(b1_composite.centerOfMass), (self.b1.centerOfMass + self.b2.centerOfMass)/2, accuracy: 0.0001)

            // inertia tensor
            XCTAssertEqual(b2_composite.inertiaTensor, self.b2.inertiaTensor, accuracy: 0.0001)
            var b1_inertiaTensor = self.b1.inertiaTensor - self.b1.mass * sqr((self.b1.centerOfMass - SIMD3<Float>(b1_composite.centerOfMass)).skew)
            b1_inertiaTensor += b2_composite.inertiaTensor - b2_composite.mass * sqr((b2_composite.centerOfMass - b1_composite.centerOfMass).skew)
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

    var root, b1, b2: ArticulatedRigidBody!
    let force: SIMD3<Float> = SIMD3<Float>(0, 1, 0) // world coordinates
    var forceAppliedPosition: float3!

    override func setUp() {
        super.setUp()

        self.device = SharedBuffersMTLDevice(MTLCreateSystemDefaultDevice()!)
        self.commandQueue = device.makeCommandQueue()!

        self.root = Tree.internode()
        self.b1 = Tree.internode()
        self.b2 = Tree.internode()
        _ = root.add(b1)
        _ = b1.add(b2)
        b2.apply(force: force)
        self.forceAppliedPosition = b2.pivot + b2.rotation.act(SIMD3<Float>(0, 1, 0))

        let (rigidBodies, rigidBodiesBuffer, ranges) = UpdateCompositeBodies.rigidBodiesBuffer(root: root, device: device)
        self.compositeBodiesBuffer = UpdateCompositeBodies.compositeBodiesBuffer(count: rigidBodies.count, device: device)
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
                    SIMD3<Float>(0,0,0.00022747851), // i.e., a small rotation about the z axis
                    SIMD3<Float>(0,0,0.02657279),
                    SIMD3<Float>(0,0,1.4540789)
                ),
                b2_parentJoint.θ, accuracy: 0.0001)

            XCTAssertEqual(
                float3x3(
                    SIMD3<Float>(0,0,8.1739134e-05), // a slightly larger torque and larger moment of inertia
                    SIMD3<Float>(0,0,0.009755036),
                    SIMD3<Float>(0,0,0.5747628)
                ),
                b1_parentJoint.θ, accuracy: 0.0001)

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

    var root, b1, b2: ArticulatedRigidBody!
    let force: SIMD3<Float> = SIMD3<Float>(0, 1, 0) // world coordinates
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
        self.forceAppliedPosition = b2.pivot + b2.rotation.act(SIMD3<Float>(0, 1, 0))

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
                SIMD3<Float>(0.70687836, 1.7073351, 0),
                SIMD3<Float>(b2.position), accuracy: 0.001)
            XCTAssertEqual(
                float3x3(
                    SIMD3<Float>(0.0011795461,-1,0),
                    SIMD3<Float>(1,0.0011795461,0),
                    SIMD3<Float>(0,0,1)),
                b2.rotation, accuracy: 0.001)

            XCTAssertEqual(
                SIMD3<Float>(0, 1, 0),
                SIMD3<Float>(b1.position))
            XCTAssertEqual(
                float3x3(
                    SIMD3<Float>(0.7073351,-0.70687836,0),
                    SIMD3<Float>(0.70687836,0.7073351,0),
                    SIMD3<Float>(0,0,1)),
                b1.rotation, accuracy: 0.001)

            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }
}

class AdvancedMetalTests: XCTestCase {
    var device: MTLDevice!
    var commandQueue: MTLCommandQueue!

    let force: SIMD3<Float> = SIMD3<Float>(0, 1, 0) // world coordinates

    var cpuSimulator: CPUSimulator!
    var metalSimulator: MetalSimulator!

    var expecteds: [ArticulatedRigidBody]!

    override func setUp() {
        super.setUp()

        self.device = SharedBuffersMTLDevice(MTLCreateSystemDefaultDevice()!)
        self.commandQueue = device.makeCommandQueue()!

        let root = Tree.internode()
        let b1 = Tree.internode()
        let b2 = Tree.internode()
        let b3 = Tree.internode()
        let b4 = Tree.internode()
        let b5 = Tree.internode()
        let b6 = Tree.internode()
        let b7 = Tree.internode()
        let b8 = Tree.internode()
        let b9 = Tree.internode()

        _ = root.add(b1)
        _ = b1.add(b2)
        _ = b1.add(b3)
        _ = b3.add(b4)
        _ = b4.add(b5)
        _ = b5.add(b6)
        _ = b5.add(b7)
        _ = b7.add(b8)
        _ = b7.add(b9)

        b9.apply(force: force)

        let world = PhysicsWorld()
        self.cpuSimulator = CPUSimulator(world: world)
        world.add(rigidBody: root)
        self.metalSimulator = MetalSimulator(device: device, root: root)
    }

    func testUpdate() {
        let expect = expectation(description: "wait")

        let commandBuffer = commandQueue.makeCommandBuffer()!

        metalSimulator.encode(commandBuffer: commandBuffer, at: 1.0/60)
        cpuSimulator.update(at: 1.0 / 60)

        commandBuffer.addCompletedHandler { _ in
            let metalSimulator = self.metalSimulator!
            let rigidBodyStructs = UnsafeMutableRawPointer(metalSimulator.rigidBodiesBuffer.contents()).bindMemory(to: RigidBodyStruct.self, capacity: metalSimulator.rigidBodies.count)
            let compositeBodies = UnsafeMutableRawPointer(metalSimulator.compositeBodiesBuffer.contents()).bindMemory(to: CompositeBodyStruct.self, capacity: metalSimulator.rigidBodies.count)
            let joints = UnsafeMutableRawPointer(metalSimulator.jointsBuffer.contents()).bindMemory(to: JointStruct.self, capacity: metalSimulator.rigidBodies.count)

            for i in 0..<(metalSimulator.rigidBodies.count-1) {
                let compositeBody = compositeBodies[i]
                let joint = joints[i]
                let rigidBody = rigidBodyStructs[i]
                let expected = metalSimulator.rigidBodies[i]

                /* FIXME
                XCTAssertEqual(expected.composite, compositeBody, accuracy: 0.01)
                if let parentJoint = expected.parentJoint {
                    XCTAssertEqual(parentJoint, joint, accuracy: 0.0001)
                }
                XCTAssertEqual(expected as! Internode, rigidBody, accuracy: 0.001)
                 */
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

    var cpuSimulator: CPUSimulator!
    var metalSimulator: MetalSimulator!

    let force: SIMD3<Float> = SIMD3<Float>(0, 1, 0) // world coordinates

    override func setUp() {
        super.setUp()
        self.device = SharedBuffersMTLDevice(MTLCreateSystemDefaultDevice()!)
        self.commandQueue = device.makeCommandQueue()!

        let root = ArticulatedRigidBody.static()
        let rigidBodyPen = RigidBodyPen(parent: root)
        let rule = Rewriter.Rule(symbol: "A", replacement: #"[!"&FA]/////[!"&FA]/////[!"&FA]"#)
        let lSystem = Rewriter.rewrite(premise: "A", rules: [rule], generations: 1)
        let configuration = InterpreterConfig(
            randomScale: 0.4,
            angle: 18 * .pi / 180,
            thickness: 0.002*0.002*Float.pi,
            thicknessScale: 0.9,
            stepSize: 0.1,
            stepSizeScale: 0.9)
        let interpreter = Interpreter(configuration: configuration, pen: rigidBodyPen)
        interpreter.interpret(lSystem)

        // FIXME
        (root.flattened().last! as! ArticulatedRigidBody).apply(force: force)

        let world = PhysicsWorld()
        self.cpuSimulator = CPUSimulator(world: world)
        world.add(rigidBody: root)
        self.metalSimulator = MetalSimulator(device: device, root: root)
    }

    func testUpdate() {
        let expect = expectation(description: "wait")

        let commandBuffer = commandQueue.makeCommandBuffer()!

        cpuSimulator.update(at: 1.0 / 60)
        metalSimulator.encode(commandBuffer: commandBuffer, at: 1.0/60)

        commandBuffer.addCompletedHandler { _ in
            let metalSimulator = self.metalSimulator!
            let rigidBodyStructs = UnsafeMutableRawPointer(metalSimulator.rigidBodiesBuffer.contents()).bindMemory(to: RigidBodyStruct.self, capacity: metalSimulator.rigidBodies.count)
            let compositeBodies = UnsafeMutableRawPointer(metalSimulator.compositeBodiesBuffer.contents()).bindMemory(to: CompositeBodyStruct.self, capacity: metalSimulator.rigidBodies.count)
            let joints = UnsafeMutableRawPointer(metalSimulator.jointsBuffer.contents()).bindMemory(to: JointStruct.self, capacity: metalSimulator.rigidBodies.count)

            for i in 0..<(metalSimulator.rigidBodies.count-1) {
                let compositeBody = compositeBodies[i]
                let joint = joints[i]
                let rigidBody = rigidBodyStructs[i]
                let expected = metalSimulator.rigidBodies[i]

                /*
                // FIXME
                XCTAssertEqual(expected as! Internode, rigidBody, accuracy: 0.001)
                XCTAssertEqual(expected.composite, compositeBody, accuracy: 0.0001)
                if let parentJoint = expected.parentJoint {
                    XCTAssertEqual(parentJoint, joint, accuracy: 0.0001)
                }
 */
            }
            expect.fulfill()
        }
        commandBuffer.commit()
        waitForExpectations(timeout: 10, handler: {error in})
    }
}
