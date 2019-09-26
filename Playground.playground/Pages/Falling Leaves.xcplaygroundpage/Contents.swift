import AppKit
import SceneKit
import PlaygroundSupport
@testable import TreePhysics

let windField = WindField(windVelocity: float3(5, 0, -1))
let gravityField = GravityField(float3(0,-9.81,0))

let world = PhysicsWorld()
let simulator = CPUSimulator(world: world)
let emitter = Emitter(birthRate: 1/5, max: 50, maxAge: 15, world: world)
let scene = Scene()
scene.add { time in
    emitter.update()
    simulator.update(at: 1.0/60)
    if let leaf = emitter.emit() {
        scene.rootNode.addChildNode(leaf.node)
    }
}

world.add(field: windField)
world.add(field: gravityField)

let scnView = SCNView(frame: CGRect(x: 0, y: 0, width: 640, height: 480))

let view = SettingsView(
    frame: CGRect(x: 0, y: 0, width: 640, height: 480 + 100))
view.add("Phi", windField.phi, 0, 2 * .pi) { phi in
    windField.phi = phi
}
view.add("airResistanceMultiplier", windField.airResistanceMultiplier, 0, 50) { airResistanceMultiplier in
    windField.airResistanceMultiplier = airResistanceMultiplier
}

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

