import AppKit
import SceneKit
import PlaygroundSupport
@testable import TreePhysics

let root = AutoTree.root()
let bud = AutoTree.Bud(position: .zero, orientation: .identity)
root.addChild(bud)

let simulator = AutoTree.GrowthSimulator()
simulator.hash.add(bud) // FIXME

simulator.attractionPoints.insert(float3(0,1.1,0))

simulator.update()

let pen = CylinderPen(radialSegmentCount: 3, heightSegmentCount: 1, parent: nil)

func draw(node: AutoTree.Node, pen: CylinderPen) {
    switch node {
    case let bud as AutoTree.Bud:
        pen.copy(scale: 0.1, orientation: bud.orientation)
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

let view = SCNView(frame: CGRect(x:0 , y:0, width: 640, height: 480))
let scene = SCNScene()

let cameraNode = SCNNode()
let camera = SCNCamera()
cameraNode.camera = camera
camera.zNear = 0
camera.zFar = 10

cameraNode.camera = camera
scene.rootNode.addChildNode(cameraNode)
cameraNode.position = SCNVector3(x: 0, y: 0.75, z: 4)

scene.rootNode.addChildNode(pen.node)
scene.rootNode.addChildNode(cameraNode)

view.scene = scene
view.backgroundColor = .black
view.showsStatistics = true
view.allowsCameraControl = true
PlaygroundPage.current.liveView = view
