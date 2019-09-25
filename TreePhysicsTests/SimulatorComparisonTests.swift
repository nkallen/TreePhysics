import XCTest
@testable import TreePhysics
import simd
import ShaderTypes

class SimulatorComparisonTests: XCTestCase {
    var device: MTLDevice!, commandQueue: MTLCommandQueue!
    var compositeBodiesBuffer, jointsBuffer, rigidBodiesBuffer: MTLBuffer!

    var cpuSimulator: CPUSimulator!
    var metalSimulator: MetalSimulator!
    var expecteds: [Internode]!

    var attractorField: AttractorField!

    override func setUp() {
        super.setUp()
        self.device = SharedBuffersMTLDevice(MTLCreateSystemDefaultDevice()!)
        self.commandQueue = device.makeCommandQueue()!

        let root = ArticulatedRigidBody.static()
        let rigidBodyPen = RigidBodyPen(parent: root)
        let rule = Rewriter.Rule(symbol: "A", replacement: #"[!"&FFFA]/////[!"&FFFA]/////[!"&FFFA]"#)
        let lSystem = Rewriter.rewrite(premise: "A", rules: [rule], generations: 1)
        let configuration = InterpreterConfiguration(
            randomScale: 0.4,
            angle: 18 * .pi / 180,
            thickness: 0.002*0.002*Float.pi,
            thicknessScale: 0.9,
            stepSize: 0.1,
            stepSizeScale: 0.9)
        let interpreter = Interpreter(configuration: configuration, pen: rigidBodyPen)
        interpreter.interpret(lSystem)

        self.attractorField = AttractorField()

        self.cpuSimulator = CPUSimulator()
        cpuSimulator.add(rigidBody: root)
        cpuSimulator.add(field: attractorField)
        self.metalSimulator = MetalSimulator(device: device, root: root)
        metalSimulator.add(field: attractorField)

        attractorField.position = float3(0.1, 0.1, 0.1)
    }

    func testUpdate() {
        let expect = expectation(description: "wait")
        tick(10, expect)
        waitForExpectations(timeout: 10, handler: {error in})
    }

    func tick(_ n: Int, _ expect: XCTestExpectation) {
        guard n > 0 else { expect.fulfill(); return }

        let commandBuffer = commandQueue.makeCommandBuffer()!

        cpuSimulator.update(at: 1.0 / 60)
        metalSimulator.encode(commandBuffer: commandBuffer, at: 1.0 / 60)
        commandBuffer.addCompletedHandler { _ in
            let metalSimulator = self.metalSimulator!

            let rigidBodies = UnsafeMutableRawPointer(metalSimulator.rigidBodiesBuffer.contents()).bindMemory(to: RigidBodyStruct.self, capacity: metalSimulator.rigidBodies.count)
            let compositeBodies = UnsafeMutableRawPointer(metalSimulator.compositeBodiesBuffer.contents()).bindMemory(to: CompositeBodyStruct.self, capacity: metalSimulator.rigidBodies.count)
            let joints = UnsafeMutableRawPointer(metalSimulator.jointsBuffer.contents()).bindMemory(to: JointStruct.self, capacity: metalSimulator.rigidBodies.count)

            for i in 0..<(metalSimulator.rigidBodies.count-1) {
                let message = "iteration[\(n)].rigidBody[\(i)]"
                XCTAssertEqual(compositeBodies[i].force,  metalSimulator.rigidBodies[i].composite.force, accuracy: 0.00001, message)
                XCTAssertEqual(compositeBodies[i].torque, metalSimulator.rigidBodies[i].composite.torque, accuracy: 0.00001, message)

                XCTAssertEqual(joints[i].θ[0],  metalSimulator.rigidBodies[i].parentJoint!.θ[0], accuracy: 0.00001, message)
                XCTAssertEqual(joints[i].θ[1],  metalSimulator.rigidBodies[i].parentJoint!.θ[1], accuracy: 0.0001, message)
                XCTAssertEqual(joints[i].θ[2],  metalSimulator.rigidBodies[i].parentJoint!.θ[2], accuracy: 0.01, message)

                XCTAssertEqual(rigidBodies[i].position, metalSimulator.rigidBodies[i].translation, accuracy: 0.0001, message)
                XCTAssertEqual(rigidBodies[i].centerOfMass, metalSimulator.rigidBodies[i].centerOfMass, accuracy: 0.0001, message)
                XCTAssertEqual(rigidBodies[i].inertiaTensor, metalSimulator.rigidBodies[i].inertiaTensor, accuracy: 0.00001, message)
            }

            self.tick(n - 1, expect)
        }

        commandBuffer.commit()
    }
}
