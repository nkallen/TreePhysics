import AppKit
import SceneKit
import PlaygroundSupport
@testable import TreePhysics

let root = RigidBody.static()
let cylinderPen = CylinderPen(radialSegmentCount: 3, heightSegmentCount: 1)
let rule = Rewriter.Rule(symbol: "A", replacement: #"[!"&FFFFFFFJA]/////[!"&FFFFFFFJA]/////[!"&FFFFFFFJA]"#)
let lSystem = Rewriter.rewrite(premise: "A", rules: [rule], generations: 5)

let configuration = Interpreter<CylinderPen>.Configuration(
    randomScale: 0.4,
    angle: 18 * .pi / 180,
    thickness: 0.002*0.002*Float.pi,
    thicknessScale: 0.7,
    stepSize: 0.1,
    stepSizeScale: 0.9)
let interpreter = Interpreter(configuration: configuration, pen: cylinderPen)
interpreter.interpret(lSystem)

let node = cylinderPen.node

let view = SCNView(frame: CGRect(x:0 , y:0, width: 640, height: 480))
let scene = SCNScene()

let cameraNode = SCNNode()
let camera = SCNCamera()
cameraNode.camera = camera
camera.zNear = 0
camera.zFar = 10

cameraNode.camera = camera
scene.rootNode.addChildNode(cameraNode)
cameraNode.position = SCNVector3(x: 0, y: 0.75, z: 4)

scene.rootNode.addChildNode(node)
scene.rootNode.addChildNode(cameraNode)

view.scene = scene
view.backgroundColor = .black
view.showsStatistics = true
view.allowsCameraControl = true
PlaygroundPage.current.liveView = view
