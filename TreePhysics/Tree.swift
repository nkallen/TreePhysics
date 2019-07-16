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

    // on rigid body:

    let mass: Float = 1
    var force: float2 = float2.zero
    var torque: float3 = float3.zero
    var inertia: Float = 0

    // NOTE: location is along the Y axis of the cylinder/branch, relative to the pivot/parent's end
    func apply(force: float2, at distance: Float) {
        self.force += force
        self.torque += cross(force, distance * float2(0, 1))
    }

    // of composite body:
    /*
     composite rigid body, we compute the mass, the world
     space inertia tensor, and the total external force and torque
     applied to the composite body evaluated about its parent
     joint.
     */

    var compositeMass: Float = 0
    var compositeInertia: Float = 0
    var compositeForce: float2 = float2.zero
    var compositeTorque: float3 = float3.zero

    func updateComposite() {
        for child in children {
            child.updateComposite()
        }
        self.compositeMass = mass + children.map { $0.compositeMass }.sum
        self.compositeForce = force + children.map { $0.rotation * $0.compositeForce }.sum
    }

    func reset() {
        self.compositeMass = 0
        self.compositeInertia = 0
        self.compositeForce = float2.zero
        self.compositeTorque = float3.zero
    }

    var rotation: float2x2 {
        return matrix2x2_rotation(radians: angle)
    }
}

extension Array where Element == Float {
    var sum: Float {
        return reduce(0, +)
    }
}

extension Array where Element == float2 {
    var sum: float2 {
        return reduce(float2.zero, +)
    }
}

func matrix2x2_rotation(radians: Float) -> float2x2 {
    let cs = cosf(radians)
    let sn = sinf(radians)
    return matrix_float2x2.init(columns:
        (float2(cs, sn),
         float2(-sn, cs)))
}

