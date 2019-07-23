import SceneKit

class GameViewController: NSViewController {
    var simulator: Simulator!
    var bone1: SCNNode!
    var bone2: SCNNode!

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
        let vertices = [float3(0.17841241, 0.0, 0.0), float3(0.17841241, 1.0, 0.0), float3(-0.089206174, 0.0, 0.1545097), float3(-0.089206174, 1.0, 0.1545097), float3(-0.089206256, 0.0, -0.15450965), float3(-0.089206256, 1.0, -0.15450965), float3(0.12615661, 1.1261566, 0.0), float3(-0.58094996, 1.8332633, 0.0), float3(-0.063078284, 0.9369217, 0.1545097), float3(-0.7701849, 1.6440284, 0.1545097), float3(-0.063078344, 0.93692166, -0.15450965), float3(-0.77018493, 1.6440284, -0.15450965)]
        let indices: [UInt8] = [0, 1, 2, 3, 4, 5, 0, 1, 1, 6, 6, 7, 8, 9, 10, 11, 6, 7]
        let geometrySource = SCNGeometrySource(vertices: vertices.map { SCNVector3($0) })
        let geometryElement = SCNGeometryElement(indices: indices, primitiveType: .triangleStrip)
        let geometry = SCNGeometry(sources: [geometrySource], elements: [geometryElement])

        let bone0 = SCNNode()
        bone0.simdPosition = float3(0,0,0)
        let bone1 = SCNNode()
        bone1.simdPosition = float3(0,1,0)
        let bone2 = SCNNode()
        bone2.simdPosition = float3(0,1,0) + normalize(float3(-1,1,0))
        let bones = [bone0, bone1, bone2]

        let boneInverseBindTransforms: [NSValue]? = bones.map { NSValue(scnMatrix4: SCNMatrix4Invert($0.transform)) }
        var boneWeights: [Float] = vertices.map { _ in 1.0 }
        var boneIndices: [UInt8] = [
            0, 1, 0, 1, 0, 1,
            1, 2, 1, 2, 1, 2,
        ]

        print(vertices.count)

        let boneWeightsData = Data(bytesNoCopy: &boneWeights, count: boneWeights.count * MemoryLayout<Float>.size, deallocator: .none)
        let boneIndicesData = Data(bytesNoCopy: &boneIndices, count: boneWeights.count * MemoryLayout<UInt8>.size, deallocator: .none)

        let boneWeightsGeometrySource = SCNGeometrySource(data: boneWeightsData, semantic: .boneWeights, vectorCount: boneWeights.count, usesFloatComponents: true, componentsPerVector: 1, bytesPerComponent: MemoryLayout<Float>.size, dataOffset: 0, dataStride: MemoryLayout<Float>.size)
        let boneIndicesGeometrySource = SCNGeometrySource(data: boneIndicesData, semantic: .boneIndices, vectorCount: boneIndices.count, usesFloatComponents: false, componentsPerVector: 1, bytesPerComponent: MemoryLayout<UInt8>.size, dataOffset: 0, dataStride: MemoryLayout<UInt8>.size)

        let skinner = SCNSkinner(baseGeometry: geometry, bones: bones, boneInverseBindTransforms: boneInverseBindTransforms, boneWeights: boneWeightsGeometrySource, boneIndices: boneIndicesGeometrySource)

        let node = SCNNode(geometry: geometry)
        node.skinner = skinner

        scnView.scene!.rootNode.addChildNode(node)
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
        bone2.simdPosition += float3(0,1,0)
//        bone1.simdPosition += float3(0,1,0)
    }

}

extension GameViewController: SCNSceneRendererDelegate {
    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        if toggle {
            Tree.gravity = float2(0, -10)
        } else {
            Tree.gravity = float2.zero
        }

//        simulator.update(at: 1.0 / 30)
        renderer.isPlaying = true
    }
}


//        let pen = CylinderPen(radialSegmentCount: 3)
//        let configuration = Interpreter.Configuration(angle: .pi/4, stepSize: 1)
//        let interpreter = Interpreter(configuration: configuration, pen: pen)
//        interpreter.interpret("""
//F[-F]
//""")
