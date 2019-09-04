import Foundation
import SceneKit

#if os(iOS)
typealias Color = UIColor
#else
typealias Color = NSColor
#endif

func createAxesNode(quiverLength: Float, quiverThickness: Float) -> SCNNode {
    let quiverThickness = (quiverLength / 50.0) * quiverThickness
    let chamferRadius = quiverThickness / 2.0

    let xQuiverBox = SCNBox(width: CGFloat(quiverLength), height: CGFloat(quiverThickness), length: CGFloat(quiverThickness), chamferRadius: CGFloat(chamferRadius))
    xQuiverBox.firstMaterial?.diffuse.contents = Color.red
    let xQuiverNode = SCNNode(geometry: xQuiverBox)
    xQuiverNode.simdPosition = float3(quiverLength / 2.0, 0.0, 0.0)

    let yQuiverBox = SCNBox(width: CGFloat(quiverThickness), height: CGFloat(quiverLength), length: CGFloat(quiverThickness), chamferRadius: CGFloat(chamferRadius))
    yQuiverBox.firstMaterial?.diffuse.contents = Color.green
    let yQuiverNode = SCNNode(geometry: yQuiverBox)
    yQuiverNode.simdPosition = float3(0.0, quiverLength / 2.0, 0.0)

    let zQuiverBox = SCNBox(width: CGFloat(quiverThickness), height: CGFloat(quiverThickness), length: CGFloat(quiverLength), chamferRadius: CGFloat(chamferRadius))
    zQuiverBox.firstMaterial?.diffuse.contents = Color.blue
    let zQuiverNode = SCNNode(geometry: zQuiverBox)
    zQuiverNode.simdPosition = float3(0.0, 0.0, quiverLength / 2.0)

    let quiverNode = SCNNode()
    quiverNode.addChildNode(xQuiverNode)
    quiverNode.addChildNode(yQuiverNode)
    quiverNode.addChildNode(zQuiverNode)
    quiverNode.name = "Axes"
    return quiverNode
}

