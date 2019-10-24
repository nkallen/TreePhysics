import AppKit
import SceneKit
import PlaygroundSupport
@testable import TreePhysics

let root = ArticulatedRigidBody.static()
let cylinderPen = CylinderPen<UInt16>(radialSegmentCount: 3)
let rigidBodyPen = RigidBodyPen(parent: root)
let skinningPen = SkinningPen(cylinderPen: cylinderPen, rigidBodyPen: rigidBodyPen)
let configuration = InterpreterConfig(
    angle: .pi / 8,
    thickness: 0.002*0.002*Float.pi,
    stepSize: 0.4,
    stepSizeScale: 0.8)

let rule = Rewriter.Rule(symbol: "A", replacement: #"[!"&FFFFFFFA&&J]////[!"&FFFFFFA&&J]////[!"&FFFFFFFFA&&J]"#)
let lSystem = Rewriter.rewrite(premise: "A", rules: [rule], generations: 3)

let interpreter = Interpreter(configuration: configuration, pen: skinningPen)
interpreter.interpret(lSystem)

let windField = WindField(windVelocity: float3(1,0,1)*13)
let gravityField = GravityField(float3(0, -9.81, 0))

let world = PhysicsWorld()
let simulator = CPUSimulator(world: world)
world.add(rigidBody: root)
world.add(field: windField)
//world.add(field: gravityField)
let scene = Scene()
scene.add { time in
    simulator.update(at: 1.0/60)
}

let view = SCNView(frame: CGRect(x:0 , y:0, width: 640, height: 480))

let cameraNode = SCNNode()
let camera = SCNCamera()
cameraNode.camera = camera
camera.zNear = 0
camera.zFar = 30

cameraNode.camera = camera
scene.rootNode.addChildNode(cameraNode)
cameraNode.position = SCNVector3(x: 5, y: 1, z: 10)
cameraNode.look(at: SCNVector3Zero)

scene.rootNode.addChildNode(skinningPen.node())
//scene.rootNode.addChildNode(skinningPen.skeleton)
scene.rootNode.addChildNode(cameraNode)

view.delegate = scene
view.scene = scene
view.backgroundColor = .black
view.showsStatistics = true
view.allowsCameraControl = true
view.isPlaying = true

PlaygroundPage.current.liveView = view
