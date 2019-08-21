import XCTest
@testable import TreePhysics
import simd

class SimulatorComparisonTests: XCTestCase {
    var device: MTLDevice!, commandQueue: MTLCommandQueue!
    var compositeBodiesBuffer, jointsBuffer, rigidBodiesBuffer: MTLBuffer!

    var cpuSimulator: Simulator!
    var metalSimulator: MetalSimulator!
    var expecteds: [RigidBody]!

    var attractorField: AttractorField!

    override func setUp() {
        super.setUp()
        self.device = MTLCreateSystemDefaultDevice()!
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
        self.attractorField = AttractorField()

        self.cpuSimulator = Simulator(tree: tree)
        cpuSimulator.add(field: attractorField)
        self.metalSimulator = MetalSimulator(device: device, root: root)
        metalSimulator.add(field: attractorField)
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
        metalSimulator.update(at: 1.0 / 60) { (compositeBodiesBuffer, jointsBuffer, rigidBodiesBuffer, rigidBodies_) in // FIXME rename
            let rigidBodies = UnsafeMutableRawPointer(rigidBodiesBuffer.contents()).bindMemory(to: RigidBodyStruct.self, capacity: rigidBodies_.count)
            let compositeBodies = UnsafeMutableRawPointer(compositeBodiesBuffer.contents()).bindMemory(to: CompositeBodyStruct.self, capacity: rigidBodies_.count)
            let joints = UnsafeMutableRawPointer(jointsBuffer.contents()).bindMemory(to: JointStruct.self, capacity: rigidBodies_.count)

            for i in 0..<(rigidBodies_.count-1) {
                XCTAssertEqual(rigidBodies[i].force, rigidBodies_[i].force)
                XCTAssertEqual(rigidBodies[i].torque, rigidBodies_[i].torque)

                XCTAssertEqual(compositeBodies[i].force,  rigidBodies_[i].composite.force)
                XCTAssertEqual(compositeBodies[i].torque, rigidBodies_[i].composite.torque)

                XCTAssertEqual(joints[i].θ,  rigidBodies_[i].parentJoint!.θ)
                XCTAssertEqual(joints[i].θ, rigidBodies_[i].parentJoint!.θ)

                XCTAssertEqual(rigidBodies[i].position, rigidBodies_[i].position)
                XCTAssertEqual(rigidBodies[i].centerOfMass, rigidBodies_[i].centerOfMass)
                XCTAssertEqual(rigidBodies[i].inertiaTensor, rigidBodies_[i].inertiaTensor, accuracy: 0.00001)
            }

            self.tick(n - 1, expect)
        }

        commandBuffer.commit()
    }
}
