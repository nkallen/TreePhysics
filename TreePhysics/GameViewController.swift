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
        let root = RigidBody(length: 0, radius: 0, density: 0, kind: .static)
        let cylinderPen = CylinderPen(radialSegmentCount: 3, heightSegmentCount: 1)
        let rigidBodyPen = RigidBodyPen(parent: root)
        let skinningPen = SkinningPen(cylinderPen: cylinderPen, rigidBodyPen: rigidBodyPen)

        let configuration = Interpreter.Configuration(thicknessScale: 0.4, stepSize: 0.2)
        let interpreter = Interpreter(configuration: configuration, pen: skinningPen)
        interpreter.interpret("""
FFFFFFFFF!"-[FFFFFFFFF!"-[FFFFFFFFF!"-[FFFFFFFFF]++[FFFFFFFFF]]++[FFFFFFFFF!"-[FFFFFFFFF]++[FFFFFFFFF]]]++[FFFFFFFFF!"-[FFFFFFFFF!"-[FFFFFFFFF]++[FFFFFFFFF]]++[FFFFFFFFF!"-[FFFFFFFFF]++[FFFFFFFFF]]]
""")
        let tree = Tree(root)
        self.simulator = Simulator(tree: tree)

        let bones = root.flatten.map { $0.node }
        let geometry = cylinderPen.geometry

        let boneInverseBindTransforms: [NSValue]? = bones.map { NSValue(scnMatrix4: SCNMatrix4Invert($0.worldTransform)) }
        var boneWeights: [Float] = cylinderPen.vertices.map { _ in 1.0 }
        var boneIndices: [UInt16] = cylinderPen.vertices.map { _ in 1 }


        let boneWeightsData = Data(bytesNoCopy: &boneWeights, count: boneWeights.count * MemoryLayout<Float>.size, deallocator: .none)
        let boneIndicesData = Data(bytesNoCopy: &boneIndices, count: boneWeights.count * MemoryLayout<UInt16>.size, deallocator: .none)

        let boneWeightsGeometrySource = SCNGeometrySource(data: boneWeightsData, semantic: .boneWeights, vectorCount: boneWeights.count, usesFloatComponents: true, componentsPerVector: 1, bytesPerComponent: MemoryLayout<Float>.size, dataOffset: 0, dataStride: MemoryLayout<Float>.size)
        let boneIndicesGeometrySource = SCNGeometrySource(data: boneIndicesData, semantic: .boneIndices, vectorCount: boneIndices.count, usesFloatComponents: false, componentsPerVector: 1, bytesPerComponent: MemoryLayout<UInt16>.size, dataOffset: 0, dataStride: MemoryLayout<UInt16>.size)

        let skinner = SCNSkinner(baseGeometry: geometry, bones: bones, boneInverseBindTransforms: boneInverseBindTransforms, boneWeights: boneWeightsGeometrySource, boneIndices: boneIndicesGeometrySource)

        let node = SCNNode(geometry: geometry)
        node.skinner = skinner

        let scene = scnView.scene!
        scene.rootNode.addChildNode(node)
        for bone in bones {
            scene.rootNode.addChildNode(bone)
        }
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
            Tree.gravity = float2(0, -100)
        } else {
            Tree.gravity = float2.zero
        }

        simulator.update(at: 1.0 / 60)
        renderer.isPlaying = true
    }
}
