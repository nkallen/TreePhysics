import Foundation
import SceneKit

fileprivate var i = 0
fileprivate let local_ijk = matrix_float4x4(diagonal: float4(1,1,1,0))

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
    private let inertiaTensor_local: float3x3
    var inertiaTensor: float3x3

    private(set) var transform: matrix_float4x4 = matrix_identity_float4x4
    var centerOfMass: float3 = float3.zero
    var force: float3 = float3.zero
    var torque: float3 = float3.zero
    let centerOfMass_local: float3
    private var rotation_local: float4x4 = matrix_identity_float4x4

    let node: SCNNode

    init(length: Float = 1.0, radius: Float = 1.0, density: Float = 1.0/Float.pi, kind: Kind = .dynamic) {
        self.name = "Branch[\(i)]"
        i += 1
        print(self.name)

        self.kind = kind

        self.mass = Float.pi * radius*radius * length * density
        self.length = length
        self.radius = radius
        let momentOfInertiaAboutY = 1.0/12 * mass * length * length // Moment of Inertia of a rod about its center of mass
        let momentOfInertiaAboutX = 1.0/4 * mass * radius * radius // MoI of a disc about its center
        let momentOfInertiaAboutZ = 1.0/4 * mass * radius * radius // ditto

        // Inertia tensor of a rod about its center of mass, see http://scienceworld.wolfram.com/physics/MomentofInertiaCylinder.html
        // and https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        self.inertiaTensor_local = matrix_float3x3.init(diagonal:
            float3(momentOfInertiaAboutY + momentOfInertiaAboutX,
                   momentOfInertiaAboutZ + momentOfInertiaAboutX,
                   momentOfInertiaAboutX + momentOfInertiaAboutY))
        self.inertiaTensor = inertiaTensor_local

        self.centerOfMass_local = float3(0, 1, 0) * length / 2

        let node = SCNNode(geometry: SCNSphere(radius: 0.01))
        self.node = node
        self.composite = CompositeBody()

        node.name = name
        node.simdPosition = position

        updateTransform()
    }

    func add(_ child: RigidBody, at eulerAngles: float3 = float3(0, 0, -.pi / 4)) {
        let joint = Joint(parent: self,
                          child: child,
                          k: kind == .static ? Float.infinity : nil)
        childJoints.append(joint)
        child.parentJoint = joint
        child.rotation_local = matrix4x4_rotation(rotation: eulerAngles)
        child.updateTransform()
    }

    // NOTE: location is along the Y axis of the cylinder/branch, relative to the pivot/parent's end
    // distance is in normalize [0..1] coordinates
    func apply(force: float3, at distance: Float) {
        guard distance >= 0 && distance <= 1 else { fatalError("Force must be applied between 0 and 1") }

        self.force += force
        self.torque += cross(convert(position: float3(0, 1, 0) * distance * length) - self.position, force)
    }

    func resetForces() {
        self.force = float3.zero
        self.torque = float3.zero
    }

    func updateTransform() {
        if let parentJoint = parentJoint {
            self.transform = parentJoint.transform * rotation_local
        } else {
            self.transform = matrix_identity_float4x4
        }
        node.simdTransform = self.transform
        let rotation_local2world = matrix3x3_rotation(from: local_ijk, to: transform)
        self.inertiaTensor = rotation_local2world * inertiaTensor_local * rotation_local2world.transpose
        self.centerOfMass = convert(position: centerOfMass_local)
    }
}

// MARK: Flattening

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

// FIXME: I think only unit tests use this for now:

extension RigidBody: Equatable, Hashable {
    static func == (lhs: RigidBody, rhs: RigidBody) -> Bool {
        return lhs === rhs
    }

    func hash(into hasher: inout Hasher) {
        hasher.combine(ObjectIdentifier(self))
    }
}

