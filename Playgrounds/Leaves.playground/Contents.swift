import AppKit
import SceneKit
import PlaygroundSupport
@testable import TreePhysics

let emitter = Emitter(max: 1000, maxAge: 20)
let simulator = Simulator(emitter: emitter)
let scene = Scene(simulator: simulator, emitter: emitter)

let scnView = SCNView(frame: CGRect(x:0 , y:0, width: 640, height: 480))

let settings: Settings = [
    ("Phi", simulator.phi, 0, 2 * Float.pi, scene,  #selector(scene.phiSliderDidChange(sender:))),
    ("airResistanceMultiplier", simulator.airResistanceMultiplier, 0, 50, scene,  #selector(scene.airResistanceMultiplierSliderDidChange(sender:)))
]

let view = SettingsView(settings: settings,
                        frame: CGRect(x: 0 , y: 0, width: 640, height: 480 + 100))

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

class Slider: CustomPlaygroundDisplayConvertible {
    var x: Float

    init(x: inout Float) {
        self.x = x
    }

    public var playgroundDescription: Any {
        print("got desc")
        let slider = NSSlider(value: Double(x), minValue: 0, maxValue: 100, target: self, action: #selector(call))
        return slider
    }

    @objc func call(sender: NSSlider) {
        print(sender.floatValue)
        self.x = sender.floatValue
    }
}

var x: Float = 10
let s = Slider(x: &x)

