import Foundation
import SceneKit
import Darwin

extension Tree {
    static let K: Float = 100
    static let B: Float = 0.03
    static let maxAngle: Float = Float.pi / 3
    static let minAngle: Float = -Float.pi / 3

    static var gravity = float2.zero
}

class Tree {
    let root: RigidBody

    init(_ root: RigidBody) {
        self.root = root
    }
}

var i = 0

class Joint: HasTransform {
    unowned let parentRigidBody: RigidBody
    let childRigidBody: RigidBody

    var transform: matrix_float3x3 = matrix_identity_float3x3
    var angularAcceleration: Float = 0
    var angularVelocity: Float = 0
    var angle: Float = 0 {
        didSet {
            node.simdRotation = float4(0, 0, 1, angle)
            updateTransform()
        }
    }

    var depth: Int {
        return parentRigidBody.depth + 1
    }

    var k: Float {
        switch depth {
        case 0: return Float.infinity
        case 1: return Tree.K * 4096
        case 2: return Tree.K * 4096
        case 3: return Tree.K * 4096
        case 4: return Tree.K * 2048
        case 5: return Tree.K * 2048
        case 6: return Tree.K * 2048
        case 7: return Tree.K * 8
        case 8: return Tree.K * 8
        case 9: return Tree.K * 8
        case 10: return Tree.K * 4
        case 11: return Tree.K * 4
        case 12: return Tree.K * 4
        case 13: return Tree.K
        case 14: return Tree.K
        default: return Tree.K
        }
    }

    let node: SCNNode

    init(parent: RigidBody, child: RigidBody) {
        self.parentRigidBody = parent
        self.childRigidBody = child
        let node = SCNNode()
        node.addChildNode(child.node)
        node.name = "Joint"
        // SceneKit's cylinder's (0,0) is at its center, so the joint is at length/2 on the y axis
        node.simdPosition = float3(0, parent.length/2, 0)
        self.node = node
        parent.childJoints.append(self)
        child.parentJoint = self
        updateTransform()
    }

    func updateTransform() {
        self.transform = parentRigidBody.transform * matrix3x3_translation(0, parentRigidBody.length) * matrix3x3_rotation(radians: self.angle)
    }
}

class RigidBody: HasTransform {
    let name: String
    weak var parentJoint: Joint?
    var childJoints: [Joint] = []

    var composite: CompositeBody!

    let mass: Float
    let length: Float
    let momentOfInertia: Float

    var transform: matrix_float3x3 = matrix_identity_float3x3 {
        didSet {
            updateCenterOfMass()
        }
    }
    var centerOfMass: float2 = float2.zero
    var force: float2 = float2.zero
    var torque: float3 = float3.zero

    let node: SCNNode

    // NOTE: the branch, in its resting state, might sprout off its parent at an angle. Thus, the
    // joint angle (which is 0 in the resting state) is not the same as the branch angle.
    var angle: Float = 0 {
        didSet {
            updateTransform()
            node.simdRotation = float4(0, 0, 1, self.angle)
        }
    }

    var depth: Int {
        if let parentJoint = parentJoint {
            return parentJoint.parentRigidBody.depth + 1
        } else {
            return 0
        }
    }

    init(mass: Float = 1.0, length: Float = 1.0) {
        self.name = "Branch[\(i)]"
        i += 1
        print(i)

        self.mass = mass
        self.length = length
        self.momentOfInertia = 1.0/12 * mass * length * length // Moment of Inertia of a rod about its center of mass

        let cylinder = SCNCylinder(radius: CGFloat(0.01), height: CGFloat(length))
        let node = SCNNode(geometry: cylinder)
        node.name = name
        node.pivot = SCNMatrix4MakeTranslation(0, CGFloat(-length / 2), 0)
        self.node = node

        self.composite = CompositeBody()

        updateCenterOfMass()
    }

    func add(_ child: RigidBody, at angle: Float = -Float.pi / 4) {
        let joint = Joint(parent: self, child: child)
        child.angle = angle
        self.node.addChildNode(joint.node)
    }

    // NOTE: location is along the Y axis of the cylinder/branch, relative to the pivot/parent's end
    // distance is in normalize [0..1] coordinates
    func apply(force: float2, at distance: Float) {
        guard distance >= 0 && distance <= 1 else { fatalError("Force must be applied between 0 and 1") }

        self.force += force
        self.torque += cross(convert(position: float2(0, 1) * distance * length) - self.position, force)
    }

    func resetForces() {
        self.force = float2.zero
        self.torque = float3.zero
    }

    func updateTransform() {
        if let parentJoint = parentJoint {
            self.transform = parentJoint.transform * matrix3x3_rotation(radians: self.angle)
        } else {
            self.transform = matrix_identity_float3x3
        }
    }

    private func updateCenterOfMass() {
        let localCenterOfMass = float2(0, 1) * length / 2
        self.centerOfMass = convert(position: localCenterOfMass)
    }
}

class CompositeBody {
    var mass: Float = 0
    var momentOfInertia: Float = 0
    var force: float2 = float2.zero
    var torque: float3 = float3.zero
    var centerOfMass: float2 = float2.zero
}
