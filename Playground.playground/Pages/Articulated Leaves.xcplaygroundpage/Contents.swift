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

let root = Internode(length: 0, radius: 0, density: 0, kind: .static)
let cylinderPen = CylinderPen(radialSegmentCount: 3, heightSegmentCount: 1)
let rigidBodyPen = RigidBodyPen(parent: root)
let skinningPen = SkinningPen(cylinderPen: cylinderPen, rigidBodyPen: rigidBodyPen)
let configuration = Interpreter<SkinningPen>.Configuration(
    angle: .pi / 10,
    thickness: 0.002*0.002*Float.pi,
    stepSize: 0.4)
let interpreter = Interpreter(configuration: configuration, pen: skinningPen)
interpreter.interpret("F+F+F+F+F+F+F+F+J")

let windField = WindField()

let simulator = CPUSimulator(root: root)
simulator.add(field: windField)
let scene = Scene(simulator: simulator)

let view = SCNView(frame: CGRect(x:0 , y:0, width: 640, height: 480))

let cameraNode = SCNNode()
let camera = SCNCamera()
cameraNode.camera = camera
camera.zNear = 0
camera.zFar = 10

cameraNode.camera = camera
scene.rootNode.addChildNode(cameraNode)
cameraNode.position = SCNVector3(x: -1, y: 0.75, z: 4)

scene.rootNode.addChildNode(skinningPen.node)
scene.rootNode.addChildNode(cameraNode)

view.delegate = scene
view.scene = scene
view.backgroundColor = .black
view.showsStatistics = true
view.allowsCameraControl = true
view.isPlaying = true

PlaygroundPage.current.liveView = view
