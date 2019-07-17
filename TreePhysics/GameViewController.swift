import SceneKit

class GameViewController: NSViewController {
    var tree: Tree!

    override func viewDidLoad() {
        super.viewDidLoad()
        
        let scene = SCNScene()
        
        let cameraNode = SCNNode()
        cameraNode.camera = SCNCamera()
        scene.rootNode.addChildNode(cameraNode)
        cameraNode.position = SCNVector3(x: 0, y: 2, z: 5)
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

    var b4: Branch!
    var b3: Branch!
    var b2: Branch!
    var b1: Branch!

    override func viewDidAppear() {
        let root = Branch()
        let b1 = Branch()
        let b2 = Branch()
        let b3 = Branch()
        let b4 = Branch()

        self.b4 = b4
        self.b3 = b3
        self.b2 = b2
        self.b1 = b1


        root.add(b1)
        b1.add(b2)
        b2.add(b3)
        b3.add(b4)
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
            let gravity: Float = 50*9.81
            b1.apply(force: float2(0,gravity * b1.mass), at: 0.5) // FIXME direction is wrong
            b2.apply(force: float2(0,gravity * b2.mass), at: 0.5) // FIXME direction is wrong
            b3.apply(force: float2(0,gravity * b3.mass), at: 0.5) // FIXME direction is wrong
            b4.apply(force: float2(0,gravity * b4.mass), at: 0.5) // FIXME direction is wrong
        }

        tree.update(delta: delta / 10)
        renderer.isPlaying = true
        previousTime = time
    }
}
