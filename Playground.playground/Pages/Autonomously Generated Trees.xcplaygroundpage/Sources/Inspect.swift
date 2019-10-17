import Foundation
@testable import TreePhysics
import simd
import SceneKit

public func draw(_ node: AutoTree.Node) -> SCNNode {
    let result = SCNNode()
    switch node {
    case let p as AutoTree.Parent:
        if let n = p.mainChild {
            let g = draw(n)
            result.addChildNode(g)
        }
        if let n = p.lateralChild {
            let g = draw(n)
            result.addChildNode(g)
        }
        let k = createAxesNode(quiverLength: 0.2, quiverThickness: 0.3)
        k.simdOrientation = p.orientation
        k.simdPosition = node.position

        let l = createAxesNode(quiverLength: 0.1, quiverThickness: 1)
        l.simdOrientation = simd_quatf(from: SIMD3<Float>(0,1,0), to: normalize(p.orientation.vertical))
        l.simdPosition = node.position
        result.addChildNode(k)
        result.addChildNode(l)
    default: ()
    }
    return result
}
