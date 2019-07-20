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
        let tree = TreeMaker().make()
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
            Tree.gravity = float2(0, -9.81)
        } else {
            Tree.gravity = float2.zero
        }

        simulator.update(at: 1.0 / 30)
        renderer.isPlaying = true
    }
}

class TreeMaker {
    static let depth = 5
    let branchAngles: [Float] = [Float.pi / 4, 0.1, -Float.pi / 3]
    let segments = 5

    static let lengthInitial: Float = 0.8 * Float(depth)
    static let radiusInitial: Float = 0.0375 * Float(depth)
    static let lengthFactor: Float = 0.6
    static let radiusFactor: Float = 0.5

    func make() -> Tree {
        let root = RigidBody(length: TreeMaker.lengthInitial, radius: TreeMaker.radiusInitial)
        make(parent: root, depth: TreeMaker.depth, length: TreeMaker.lengthInitial * TreeMaker.lengthFactor, radius: TreeMaker.radiusInitial * TreeMaker.radiusFactor)
        return Tree(root)
    }

    private func make(parent: RigidBody, depth: Int, length: Float, radius: Float) {
        guard depth >= 0 else { return }
        for branchAngle in branchAngles {
            let branch = RigidBody(length: length / Float(segments), radius: radius)
            parent.add(branch, at: branchAngle)
            var segment = branch
            for _ in 1..<segments {
                let branch = RigidBody(length: length / Float(segments), radius: radius)
                segment.add(branch, at: 0)
                segment = branch
            }
            make(parent: segment, depth: depth - 1, length: length * TreeMaker.lengthFactor, radius: radius * TreeMaker.radiusFactor)
        }
    }
}
