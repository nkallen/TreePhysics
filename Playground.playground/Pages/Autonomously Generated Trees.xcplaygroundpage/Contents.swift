import AppKit
import SceneKit
import PlaygroundSupport
import SceneKit.ModelIO
@testable import TreePhysics

let autoTree = AutoTree()

let root = autoTree.seedling()
let simulator = autoTree.growthSimulator()
simulator.add(root)

let url = Bundle.main.url(forResource: "ARFaceGeometry", withExtension: "obj", subdirectory: "art.scnassets")!
let asset = MDLAsset(url: url)
let mdlMesh = asset.object(at: 0) as! MDLMesh

let face = SCNNode(mdlObject: mdlMesh)
let scale: Float = 0.008
let offset = float3(0,0.5,0)
face.simdScale = float3(repeating: 1) * scale
face.simdPosition = offset

simulator.attractionPoints.formUnion(mdlMesh.vertices.map { $0 * scale + offset })

for _ in 0...35 { simulator.update() }

let pen = CylinderPen<UInt16>(radialSegmentCount: 3, heightSegmentCount: 1, parent: nil)

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
