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
    var children: [Branch] = []
    weak var parent: Branch? {
        didSet {
            self.angle = -Float.pi / 4
        }
    }
    let name: String

    var angle: Float = 0 {
        didSet {
            node.simdRotation = float4(0, 0, 1, self.angle)
        }
    }

    init() {
        self.name = "Branch[\(i)]"
        i += 1
    }

    lazy var node: SCNNode = {
        let cylinder = SCNCylinder(radius: CGFloat(0.1), height: CGFloat(1))
        let node = SCNNode(geometry: cylinder)
        node.name = name

        node.pivot = SCNMatrix4MakeTranslation(0, -0.5, 0)

        node.simdPosition = float3(0,0.5,0)

        return node
    }()

    func add(_ child: Branch) {
        child.parent = self
        self.children.append(child)
        self.node.addChildNode(child.node)
    }
}
