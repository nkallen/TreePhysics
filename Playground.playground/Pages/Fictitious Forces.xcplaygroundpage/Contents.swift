import AppKit
import SceneKit
import PlaygroundSupport
@testable import TreePhysics

let root = ArticulatedRigidBody.static()
let cylinderPen = CylinderPen(radialSegmentCount: 3, heightSegmentCount: 1)
let rigidBodyPen = RigidBodyPen(parent: root)
let skinningPen = SkinningPen(cylinderPen: cylinderPen, rigidBodyPen: rigidBodyPen)
do {
    let configuration = InterpreterConfig(
        randomScale: 0.4,
        angle: 18 * .pi / 180,
        thickness: 0.002*0.002*Float.pi,
        thicknessScale: 0.7,
        stepSize: 0.1,
        stepSizeScale: 0.9)
    let rule = Rewriter.Rule(symbol: "A", replacement: #"[!"&FFFFFFFJA]/////[!"&FFFFFFFJA]/////[!"&FFFFFFFJA]"#)
    let lSystem = Rewriter.rewrite(premise: "A", rules: [rule], generations: 3)
    let interpreter = Interpreter(configuration: configuration, pen: skinningPen)
    interpreter.interpret(lSystem)
}

let configuration = SimulatorConfig(
    torqueFictitiousMultiplier_i: 1,
    torqueFictitiousMultiplier_ii: 1,
    torqueFictitiousMultiplier_iii: 1)
let simulator = CPUSimulator(configuration: configuration)
simulator.add(rigidBody: root)
let scene = Scene()
scene.add { time in
    simulator.update(at: 1/60)
}
root.acceleration = float3(5,0,0)
root.angularVelocity = float3(0,5,0)
root.angularAcceleration = float3(0,5,0)

let view = SCNView(frame: CGRect(x:0 , y:0, width: 640, height: 480))

let cameraNode = SCNNode()
let camera = SCNCamera()
cameraNode.camera = camera

cameraNode.camera = camera
scene.rootNode.addChildNode(cameraNode)
cameraNode.position = SCNVector3(x: 0, y: 1, z: 2)

scene.rootNode.addChildNode(skinningPen.node)
scene.rootNode.addChildNode(cameraNode)

view.delegate = scene
view.scene = scene
view.backgroundColor = .black
view.showsStatistics = true
view.allowsCameraControl = true
view.isPlaying = true

PlaygroundPage.current.liveView = view
