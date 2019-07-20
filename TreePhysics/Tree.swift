import Foundation
import SceneKit
import Darwin

extension Tree {
    static let K: Float = 100
    static let B: Float = 0.02
    static let maxAngle: Float = Float.pi / 3
    static let minAngle: Float = -Float.pi / 3

    static var gravity = float2.zero
}

final class Tree {
    let root: RigidBody

    init(_ root: RigidBody) {
        self.root = root
    }
}

var i = 0

final class Joint: HasTransform {
    unowned let parentRigidBody: RigidBody
    let childRigidBody: RigidBody

    var transform: matrix_float3x3 = matrix_identity_float3x3
    var angularAcceleration: Float = 0
    var angularVelocity: Float = 0
    var angle: Float = 0 {
        didSet {
            node.simdRotation = float4(0, 0, 1, angle)
            self.transform = parentRigidBody.transform * matrix3x3_translation(0, parentRigidBody.length) * matrix3x3_rotation(radians: self.angle)
        }
    }

    let k: Float

    let node: SCNNode

    init(parent: RigidBody, child: RigidBody) {
        self.parentRigidBody = parent
        self.childRigidBody = child
        self.k = Joint.computeK(radius: parent.radius)
        print(parent.radius, self.k)

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

    private static func computeK(radius: Float) -> Float {
        if radius <= 0.003 {
            return Tree.K
        } else if radius <= 0.005 {
            return Tree.K
        } else if radius <= 0.01 {
            return 4 * Tree.K
        } else if radius <= 0.02 {
            return 256 * Tree.K
        } else if radius <= 0.04 {
            return 256 * Tree.K
        } else if radius <= 0.075 {
            return 1024 * Tree.K
        } else if radius <= 0.15 {
            return 4096 * Tree.K
        } else {
            return 16384 * Tree.K
        }
    }
}

final class RigidBody: HasTransform {
    let name: String
    weak var parentJoint: Joint?
    var childJoints: [Joint] = []

    let composite: CompositeBody

    let mass: Float
    let length: Float
    let radius: Float
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

    let density: Float = 750

    init(length: Float = 1.0, radius: Float = 1.0) {
        self.name = "Branch[\(i)]"
        print(name)
        i += 1

        self.mass = Float.pi * radius*radius * length * density
        self.length = length
        self.radius = radius
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

final class CompositeBody {
    var mass: Float = 0
    var momentOfInertia: Float = 0
    var force: float2 = float2.zero
    var torque: float3 = float3.zero
    var centerOfMass: float2 = float2.zero
}
