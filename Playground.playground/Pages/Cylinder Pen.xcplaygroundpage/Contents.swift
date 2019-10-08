import AppKit
import SceneKit
import PlaygroundSupport
import SceneKit.ModelIO
@testable import TreePhysics

let pen = CylinderPen<UInt16>(radialSegmentCount: 10, parent: nil)

pen.start(at: .zero, orientation: .identity, thickness: 1)
pen.cont(distance: 1, orientation: simd_quatf(angle: .pi/4, axis: .z), thickness: 0.1)
pen.copy(scale: 1, orientation: .identity)
let branch = pen.branch()
pen.cont(distance: 1, orientation: .identity, thickness: 0.3)
branch.cont(distance: 1, orientation: simd_quatf(angle: .pi/2, axis: .x), thickness: 0.2)
pen.copy(scale: 1, orientation: .identity)

let view = SCNView(frame: CGRect(x:0 , y:0, width: 640, height: 480))
let scene = SCNScene()

scene.rootNode.addChildNode(pen.node())

view.scene = scene
view.backgroundColor = .gray
view.showsStatistics = true
view.allowsCameraControl = true
PlaygroundPage.current.liveView = view
