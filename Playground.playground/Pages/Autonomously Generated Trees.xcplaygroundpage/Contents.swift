import AppKit
import SceneKit
import PlaygroundSupport
import SceneKit.ModelIO
@testable import TreePhysics

var config = AutoTree.Config()
config.internodeLength = 0.03
config.occupationRadius = 0.03
config.perceptionRadius = 0.3
config.biasVigorTowardsMainAxis = 0.54
let autoTree = AutoTree(config)

let (root, _) = autoTree.seedling()
let simulator = autoTree.growthSimulator()
simulator.add(root)

let url = Bundle.main.url(forResource: "ARFaceGeometry", withExtension: "obj", subdirectory: "art.scnassets")!
let asset = MDLAsset(url: url)
let mdlMesh = asset.object(at: 0) as! MDLMesh

let face = SCNNode(mdlObject: mdlMesh)
let scale: Float = 0.01
let offset = float3(0,1,-0.1)
face.simdScale = float3(repeating: 1) * scale
face.simdPosition = offset

simulator.attractionPoints.formUnion(mdlMesh.vertices.map { $0 * scale + offset })

for _ in 0...100 { simulator.update() }

let pen = CylinderPen<UInt32>(radialSegmentCount: 15, parent: nil)

autoTree.draw(root, pen: pen)

let view = SCNView(frame: CGRect(x:0 , y:0, width: 640, height: 480))
let scene = SCNScene()


scene.rootNode.addChildNode(pen.node())
//scene.rootNode.addChildNode(face)

view.scene = scene
view.backgroundColor = .gray
view.showsStatistics = true
view.allowsCameraControl = true
PlaygroundPage.current.liveView = view
