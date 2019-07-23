import Foundation
import SceneKit

var i = 0

final class RigidBody: HasTransform {
    enum Kind {
        case `static`
        case `dynamic`
    }
    let kind: Kind

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
            node.simdPosition = float3(convert(position: float2(0,0)), 0)
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
        }
    }

    init(length: Float = 1.0, radius: Float = 1.0, density: Float = 1.0/Float.pi, kind: Kind = .dynamic) {
        self.name = "Branch[\(i)]"
        i += 1

        self.kind = kind

        self.mass = Float.pi * radius*radius * length * density
        self.length = length
        self.radius = radius
        self.momentOfInertia = 1.0/12 * mass * length * length // Moment of Inertia of a rod about its center of mass

        let node = SCNNode(geometry: SCNSphere(radius: 0.1))
        self.node = node
        self.composite = CompositeBody()

        node.name = name
        node.simdPosition = float3(convert(position: float2(0,0)),0)

        updateCenterOfMass()
    }

    func add(_ child: RigidBody, at angle: Float = -Float.pi / 4) {
        let joint: Joint
        if kind == .static {
            joint = Joint(parent: self, child: child, k: Float.infinity)
        } else {
            joint = Joint(parent: self, child: child)
        }
        childJoints.append(joint)
        child.angle = angle
        child.parentJoint = joint
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

extension RigidBody {
    var flatten: [RigidBody] {
        var result: [RigidBody] = []
        var queue: [RigidBody] = [self]
        searchBreadthFirst(queue: &queue, result: &result)
        return result
    }

    private func searchBreadthFirst(queue: inout [RigidBody], result: inout [RigidBody]) {
        while !queue.isEmpty {
            let start = queue.removeFirst()
            result.append(start)
            for childJoint in start.childJoints {
                queue.append(childJoint.childRigidBody)
            }
        }
    }
}
