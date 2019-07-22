import SceneKit

class GameViewController: NSViewController {
    var simulator: Simulator!

    override func viewDidLoad() {
        super.viewDidLoad()
        
        let scene = SCNScene()
        
        let cameraNode = SCNNode()
        let camera = SCNCamera()
        camera.zNear = 0
        cameraNode.camera = camera
        scene.rootNode.addChildNode(cameraNode)
        cameraNode.position = SCNVector3(x: 0, y: 3, z: 8)
        cameraNode.name = "Camera"

        let ambientLightNode = SCNNode()
        ambientLightNode.light = SCNLight()
        ambientLightNode.light!.type = .ambient
        ambientLightNode.name = "Ambient Light"
        scene.rootNode.addChildNode(ambientLightNode)
        
        scnView.scene = scene
        scnView.allowsCameraControl = true
        scnView.showsStatistics = true
        scnView.backgroundColor = NSColor.black

        let clickGesture = NSClickGestureRecognizer(target: self, action: #selector(handleClick(_:)))
        var gestureRecognizers = scnView.gestureRecognizers
        gestureRecognizers.insert(clickGesture, at: 0)
        scnView.gestureRecognizers = gestureRecognizers
    }

    override func viewDidAppear() {
        let root = RigidBody(length: 0, radius: 0, kind: .static)
        let pen = CylinderPen(parent: root, angle: .pi/2)
        let configuration = Interpreter.Configuration(thicknessScale: 0.4, stepSize: 0.5)
        let interpreter = Interpreter(configuration: configuration, pen: pen)
        interpreter.interpret("""
FFFFFFFFF!"-[FFFFFFFFF!"-[FFFFFFFFF!"-[FFFFFFFFF]++[FFFFFFFFF]]++[FFFFFFFFF!"-[FFFFFFFFF]++[FFFFFFFFF]]]++[FFFFFFFFF!"-[FFFFFFFFF!"-[FFFFFFFFF]++[FFFFFFFFF]]++[FFFFFFFFF!"-[FFFFFFFFF]++[FFFFFFFFF]]]
""")
        let tree = Tree(root)
        self.simulator = Simulator(tree: tree)
        scnView.scene!.rootNode.addChildNode(tree.root.node)
        scnView.delegate = self
    }

    var scnView: SCNView {
        return self.view as! SCNView
    }
}

var toggle: Bool = false

extension GameViewController {
    @objc
    func handleClick(_ gestureRecognizer: NSGestureRecognizer) {
        toggle = !toggle
    }

}

extension GameViewController: SCNSceneRendererDelegate {
    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        if toggle {
            Tree.gravity = float2(0, -10)
        } else {
            Tree.gravity = float2.zero
        }

        simulator.update(at: 1.0 / 30)
        renderer.isPlaying = true
    }
}
