import AppKit
import SceneKit
import PlaygroundSupport
import Cocoa
@testable import TreePhysics

let colors: [NSColor] = [
    NSColor(red: 107/255, green: 147/255, blue: 77/255, alpha: 1),
    NSColor(red: 202/255, green: 174/255, blue: 110/255, alpha: 1),
    NSColor(red: 141/255, green: 154/255, blue: 57/255, alpha: 1),
    NSColor(red: 212/255, green: 97/255, blue: 93/255, alpha: 1),
]

let images: [NSImage] = ["leaf1", "leaf2", "leaf3"].map { name in
    let url = Bundle.main.url(forResource: name, withExtension: "png", subdirectory: "art.scnassets")!
    let image = NSImage(byReferencing: url)
    return image
}

var materials: [SCNMaterial] = []
for color in colors {
    for image in images {
        let material = SCNMaterial()
        material.diffuse.contents = image
        material.multiply.contents = color

        materials.append(material)
    }
}

//let asset = MDLAsset(url: url)
//let mdlMesh = asset.object(at: 0) as! MDLMesh

let windField = WindField(windVelocity: simd_float3(5, 0, -1))
let gravityField = GravityField(simd_float3(0,-9.81,0))

let world = PhysicsWorld()
let simulator = CPUSimulator(world: world)
let emitter = Emitter(birthRate: 1/5, max: 30, maxAge: 15, world: world)
let scene = Scene()
scene.add { time in
    emitter.update()
    simulator.update(at: 1.0/60)
    if let leaf = emitter.emit() {
        let node = leaf.node
        let material = materials.randomElement()!
        node.geometry?.firstMaterial = material

        scene.rootNode.addChildNode(node)
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

