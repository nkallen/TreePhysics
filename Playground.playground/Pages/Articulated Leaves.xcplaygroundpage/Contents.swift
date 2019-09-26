import AppKit
import SceneKit
import PlaygroundSupport
@testable import TreePhysics

let delta: Double = 1/60

public class Scene: SCNScene, SCNSceneRendererDelegate {
    let simulator: CPUSimulator

    public init(simulator: CPUSimulator) {
        self.simulator = simulator
        super.init()
    }

    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }

    public func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        simulator.update(at: delta)
    }
}

let root = ArticulatedRigidBody.static()
let cylinderPen = CylinderPen(radialSegmentCount: 3, heightSegmentCount: 1)
let rigidBodyPen = RigidBodyPen(parent: root)
let skinningPen = SkinningPen(cylinderPen: cylinderPen, rigidBodyPen: rigidBodyPen)
let configuration = InterpreterConfig(
    angle: .pi / 8,
    thickness: 0.002*0.002*Float.pi,
    stepSize: 0.4,
    stepSizeScale: 0.8)

let rule = Rewriter.Rule(symbol: "A", replacement: #"[!"&FFFFFFFA&&J]////[!"&FFFFFFFA&&J]////[!"&FFFFFFFA&&J]"#)
let lSystem = Rewriter.rewrite(premise: "A", rules: [rule], generations: 2)

let interpreter = Interpreter(configuration: configuration, pen: skinningPen)
interpreter.interpret(lSystem)

let windField = WindField(windVelocity: float3(1,0,1)*14)
let gravityField = GravityField(float3(0, -9.81, 0))

let simulator = CPUSimulator()
simulator.add(rigidBody: root)
simulator.add(field: windField)
simulator.add(field: gravityField)
let scene = Scene(simulator: simulator)

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

scene.rootNode.addChildNode(skinningPen.node)
scene.rootNode.addChildNode(cameraNode)

view.delegate = scene
view.scene = scene
view.backgroundColor = .black
view.showsStatistics = true
view.allowsCameraControl = true
view.isPlaying = true

PlaygroundPage.current.liveView = view
