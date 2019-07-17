import SceneKit

class GameViewController: NSViewController {
    var tree: Tree!

    override func viewDidLoad() {
        super.viewDidLoad()
        
        let scene = SCNScene()
        
        let cameraNode = SCNNode()
        cameraNode.camera = SCNCamera()
        scene.rootNode.addChildNode(cameraNode)
        cameraNode.position = SCNVector3(x: 0, y: 1, z: 5)
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
        scnView.delegate = self
    }

    override func viewDidAppear() {
        let root = Branch()
        let b1 = Branch()
        let b2 = Branch()

        root.add(b1)
        b1.add(b2)
        self.tree = Tree(root)

        b2.apply(force: float2(0,1), at: 1)
        tree.root.updateCompositeBodyState()

        scnView.scene!.rootNode.addChildNode(tree.root.node)
    }

    var scnView: SCNView {
        return self.view as! SCNView
    }
}

var previousTime: TimeInterval? = nil

extension GameViewController: SCNSceneRendererDelegate {
    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        let delta = time - (previousTime ?? time)
        tree.update(delta: delta)
    }
}
