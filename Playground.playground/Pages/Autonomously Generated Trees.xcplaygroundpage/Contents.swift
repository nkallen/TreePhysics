import AppKit
import SceneKit
import PlaygroundSupport
import SceneKit.ModelIO
@testable import TreePhysics

let root = AutoTree.root()
let bud = AutoTree.TerminalBud(position: .zero, orientation: .identity)
root.addChild(bud)

let simulator = AutoTree.GrowthSimulator()
simulator.hash.add(bud) // FIXME

let url = Bundle.main.url(forResource: "ARFaceGeometry", withExtension: "obj", subdirectory: "art.scnassets")!
let asset = MDLAsset(url: url)
let mdlMesh = asset.object(at: 0) as! MDLMesh

let face = SCNNode(mdlObject: mdlMesh)
face.simdScale = float3(0.005, 0.005, 0.005)
face.simdPosition = float3(0,0.5,0)

simulator.attractionPoints.formUnion(mdlMesh.vertices.map { $0 * 0.005 + float3(0,0.5,0) })


for i in 0...15 { simulator.update() }

let pen = CylinderPen<UInt16>(radialSegmentCount: 3, heightSegmentCount: 1, parent: nil)

func draw<I>(node: AutoTree.Node, pen: CylinderPen<I>) {
    switch node {
    case let bud as AutoTree.Bud: ()
//        pen.copy(scale: 0.01, orientation: bud.orientation)
    case let internode as AutoTree.Internode:
        pen.cont(distance: internode.length, orientation: internode.orientation, thickness: sqr(internode.radius) * .pi)
    default:
        pen.start(at: node.position, thickness: 1)
    }

    // Reorganize branching structure following thickest path topology,
    // cf, [Longay 2014], appendix C.3
    let (thickest, rest) = node.children.thickest
    for child in rest {
        let branch = pen.branch()
        draw(node: child, pen: branch)
    }
    if let thickest = thickest { draw(node: thickest, pen: pen) }
}

fileprivate extension Set where Element == AutoTree.Node {
    var thickest: (AutoTree.Node?, Set<AutoTree.Node>) {
        var rest: Set<AutoTree.Node> = []
        var thickest: AutoTree.Internode? = nil
        for node in self {
            switch (thickest, node) {
            case let (nil, internode as AutoTree.Internode):
                thickest = internode
            case let (.some(last), internode as AutoTree.Internode) where internode.radius > last.radius:
                rest.insert(last)
                thickest = internode
            default:
                rest.insert(node)
            }
        }
        return (thickest, rest)
    }
}

draw(node: root, pen: pen)


///////////

let view = SCNView(frame: CGRect(x:0 , y:0, width: 640, height: 480))
let scene = SCNScene()

scene.rootNode.addChildNode(pen.node)
scene.rootNode.addChildNode(face)

view.scene = scene
view.backgroundColor = .gray
view.showsStatistics = true
view.allowsCameraControl = true
PlaygroundPage.current.liveView = view
