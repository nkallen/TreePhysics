import Foundation
import SceneKit

class Tree {
    let root: Branch

    init(_ root: Branch) {
        self.root = root
    }
}

var i = 0

class Branch {
    let position: float3 = float3(0,0.5,0)
    var children: [Branch] = []
    weak var parent: Branch?
    let name: String

    init() {
        self.name = "Branch[\(i)]"
        i += 1
    }

    lazy var node: SCNNode = {
        let cylinder = SCNCylinder(radius: CGFloat(0.1), height: CGFloat(1))
        let node = SCNNode(geometry: cylinder)
        node.name = name

        node.pivot = SCNMatrix4MakeTranslation(0, -0.5, 0)

        node.simdPosition = position

        if parent != nil {
            node.simdRotation = float4(0, 0, -Float.pi / 4, 1)
        }
        return node
    }()

    func add(_ child: Branch) {
        child.parent = self
        self.children.append(child)
        self.node.addChildNode(child.node)
    }
}
