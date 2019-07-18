import SceneKit

class GameViewController: NSViewController {
    var tree: Tree!

    override func viewDidLoad() {
        super.viewDidLoad()
        
        let scene = SCNScene()
        
        let cameraNode = SCNNode()
        let camera = SCNCamera()
        camera.zNear = 0
        cameraNode.camera = camera
        scene.rootNode.addChildNode(cameraNode)
        cameraNode.position = SCNVector3(x: 0, y: 0.3, z: 0.5)
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

        // Add a click gesture recognizer
        let clickGesture = NSClickGestureRecognizer(target: self, action: #selector(handleClick(_:)))
        var gestureRecognizers = scnView.gestureRecognizers
        gestureRecognizers.insert(clickGesture, at: 0)
        scnView.gestureRecognizers = gestureRecognizers
    }

    var b4: RigidBody!
    var b3: RigidBody!
    var b2: RigidBody!
    var b1: RigidBody!

    override func viewDidAppear() {
        let root = RigidBody(mass: 1, length: 0.1)
        let b1 = RigidBody(mass: 0.5, length: 0.1)
        let b2 = RigidBody(mass: 1.0/2, length: 0.1)
        let b3 = RigidBody(mass: 1.0/3, length: 0.1)
        let b4 = RigidBody(mass: 1.0/4, length: 0.1)

        self.b4 = b4
        self.b3 = b3
        self.b2 = b2
        self.b1 = b1


        root.add(b1, at: -Float.pi / 8)
        b1.add(b2, at: -Float.pi / 8)
        b2.add(b3, at: -Float.pi / 8)
        b3.add(b4, at: -Float.pi / 8)
        self.tree = Tree(root)
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

var previousTime: TimeInterval? = nil

extension GameViewController: SCNSceneRendererDelegate {
    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        let delta = time - (previousTime ?? time)
        print(delta)
        if toggle {
            let gravity: Float = -9.81
            b1.apply(force: float2(0,gravity * b1.mass), at: 0.5) // FIXME direction is wrong
            b2.apply(force: float2(0,gravity * b2.mass), at: 0.5) // FIXME direction is wrong
            b3.apply(force: float2(0,gravity * b3.mass), at: 0.5) // FIXME direction is wrong
            b4.apply(force: float2(0,gravity * b4.mass), at: 0.5) // FIXME direction is wrong
        }

        tree.update(delta: delta)
        renderer.isPlaying = true
        previousTime = time
    }
}
