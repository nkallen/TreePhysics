import Foundation
import SceneKit
import ShaderTypes
import TreePhysics

class Scene: NSObject {
    let scene: SCNScene
//    let gravityField: GravityField
//    let attractorField: AttractorField
//    let attractor: SCNNode
    let root: ArticulatedRigidBody

//    private let metalSimulator: MetalSimulator
    private let cpuSimulator: CPUSimulator

    let device: MTLDevice, commandQueue: MTLCommandQueue

    override init() {
        self.device = MTLCreateSystemDefaultDevice()!
        self.commandQueue = device.makeCommandQueue()!

        self.scene = SCNScene()
        
        let cameraNode = SCNNode()
        let camera = SCNCamera()
        camera.zNear = 0
        camera.zFar = 100
        cameraNode.camera = camera
        scene.rootNode.addChildNode(cameraNode)
        cameraNode.position = SCNVector3(x: 0, y: 0.75, z: 5.5)
        cameraNode.name = "Camera"
        
        let ambientLightNode = SCNNode()
        ambientLightNode.light = SCNLight()
        ambientLightNode.light!.type = .ambient
        ambientLightNode.name = "Ambient Light"
        scene.rootNode.addChildNode(ambientLightNode)
        
        // Tree:
        
        self.root = ArticulatedRigidBody.static()
        let cylinderPen = CylinderPen<UInt16>(radialSegmentCount: 3)
        let rigidBodyPen = RigidBodyPen(parent: root)
        let skinningPen = SkinningPen(cylinderPen: cylinderPen, rigidBodyPen: rigidBodyPen)
        
        let rule = Rewriter.Rule(symbol: "A", replacement: #"[!"&FFFFFA&&J]////[!"&FFFFFA&&J]////[!"&FFFFFA&&J]"#)
        let lSystem = Rewriter.rewrite(premise: "A", rules: [rule], generations: 5)

        let configuration = InterpreterConfig(
            randomScale: 0.4,
            angle: 18 * .pi / 180,
            thickness: 0.002*0.002*Float.pi,
            thicknessScale: 0.7,
            stepSize: 0.3,
            stepSizeScale: 0.9)
        let interpreter = Interpreter(configuration: configuration, pen: skinningPen)
        interpreter.interpret(lSystem)
        let world = PhysicsWorld()
        self.cpuSimulator = CPUSimulator(world: world)
        world.add(rigidBody: root)
        
        scene.rootNode.addChildNode(skinningPen.node())
//        scene.rootNode.addChildNode(skinningPen.skeleton)

        scene.rootNode.addChildNode(createAxesNode(quiverLength: 1, quiverThickness: 0.25))

        // Forces:
        let gravityField = GravityField(float3.zero)
//        let attractorField = AttractorField()

//        let attractor = SCNNode(geometry: SCNSphere(radius: 0.1))
//        scene.rootNode.addChildNode(attractor)

//        self.gravityField = gravityField
//        self.attractorField = attractorField
//        self.attractor = attractor

//        self.metalSimulator = MetalSimulator(device: device, root: root)

//        cpuSimulator.add(field: gravityField)
//        cpuSimulator.add(field: attractorField)
//        metalSimulator.add(field: attractorField)

        let windField = WindField(windVelocity: float3(1,0,1)*15)
        world.add(field: windField)
        world.add(field: gravityField)
    }
}

var start = Date()

let radius: Float = 3
var done: Bool = false

extension Scene: SCNSceneRendererDelegate {
    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        //        let pov = renderer.pointOfView!
//        pov.simdPosition = float3(
//            radius * sinf(Float(start.timeIntervalSinceNow)),
//            1,
//            radius * cosf(Float(start.timeIntervalSinceNow)))
//        pov.look(at: SCNVector3(0,1,0), up: SCNVector3(0,1,0), localFront: SCNVector3(0,0,-1))

        cpuSimulator.update(at: 1/60)
        renderer.isPlaying = true

//        return ()
//        let commandBuffer = commandQueue.makeCommandBuffer()!
//        metalSimulator.encode(commandBuffer: commandBuffer, at: 1.0 / 100)
//        commandBuffer.addCompletedHandler { [unowned self] _ in
//            let metalSimulator = self.metalSimulator
//            DispatchQueue.main.async {
//                let rigidBodies = UnsafeMutableRawPointer(metalSimulator.rigidBodiesBuffer.contents()).bindMemory(to: RigidBodyStruct.self, capacity: metalSimulator.rigidBodies.count)
//
//                for i in 0..<(metalSimulator.rigidBodies.count-1) {
//                    metalSimulator.rigidBodies[i].node.simdPosition = rigidBodies[i].position
//                }
//            }
//        }
//        commandBuffer.commit()
    }
}
