import AppKit
import SceneKit
import PlaygroundSupport
@testable import TreePhysics

let windField = WindField(windVelocity: float3(5, 0, -1))
let gravityField = GravityField(float3(0,-9.81,0))

let simulator = CPUSimulator()
let emitter = Emitter(max: 1000, maxAge: 20, simulator: simulator)
let scene = Scene(simulator: simulator, emitter: emitter, windField: windField)

simulator.add(field: windField)
simulator.add(field: gravityField)

let scnView = SCNView(frame: CGRect(x: 0, y: 0, width: 640, height: 480))

let settings: Settings = [
    ("Phi", windField.phi, 0, 2 * Float.pi, scene,  #selector(scene.phiSliderDidChange(sender:))),
    ("airResistanceMultiplier", windField.airResistanceMultiplier, 0, 50, scene,  #selector(scene.airResistanceMultiplierSliderDidChange(sender:)))
]

let view = SettingsView(settings: settings,
                        frame: CGRect(x: 0, y: 0, width: 640, height: 480 + 100))

view.addArrangedSubview(scnView)

let cameraNode = SCNNode()
let camera = SCNCamera()
cameraNode.camera = camera
camera.zNear = 0
camera.zFar = 1000
cameraNode.camera = camera
cameraNode.position = SCNVector3(x: 0, y: -25, z: 50)
scene.rootNode.addChildNode(cameraNode)

scnView.delegate = scene
scnView.scene = scene
scnView.backgroundColor = .black
scnView.showsStatistics = true
scnView.allowsCameraControl = true
scnView.isPlaying = true

PlaygroundPage.current.liveView = view

