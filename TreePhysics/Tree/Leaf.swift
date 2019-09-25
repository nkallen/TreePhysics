import Foundation
import SceneKit
import simd

extension Leaf {
    static let thickness: Float = 0.001
}

final class Leaf: RigidBody {
    let kind: Kind = .dynamic

    // FIXME is this right?
    let childJoints: [Joint] = []

    weak var parentJoint: Joint?
    let composite: CompositeBody

    let mass: Float
    var inertiaTensor: float3x3

    var force: float3 = float3.zero
    var torque: float3 = float3.zero

    var centerOfMass: float3 = float3.zero
    var rotation: simd_quatf = simd_quatf.identity
    var translation: float3 = float3.zero

    var angularVelocity: float3 = float3.zero
    var angularAcceleration: float3 = float3.zero
    var angularMomentum: float3 = float3.zero
    var velocity: float3 = float3.zero
    var acceleration: float3 = float3.zero

    let node: SCNNode

    let inertiaTensor_local: float3x3
    let centerOfMass_local: float3
    let length: Float

    let area: Float

    // FIXME
    var normal: float3 {
        return rotation.act(float3.z)
    }

    init(length: Float = 1.0, density: Float = 1.0) {
        self.length = length
        self.area = sqr(length)
        self.mass = density * area * Leaf.thickness

        // Inertia tensor for rectangular plate:
        self.inertiaTensor_local = float3x3(diagonal:
            float3(1/12 * mass * sqr(length),
                   1/12 * mass * sqr(length),
                   1/6  * mass * sqr(length)))

        self.inertiaTensor = inertiaTensor_local

        self.centerOfMass_local = float3(length/2, length/2, 0)

        let plane = SCNBox(width: CGFloat(length), height: CGFloat(length), length: CGFloat(0.01), chamferRadius: 0)
        plane.firstMaterial?.isDoubleSided = true
        self.node = SCNNode(geometry: plane)
        self.composite = CompositeBody()

        updateTransform()
    }

    func apply(force: float3, torque: float3?) {
        self.force += force
        if let torque = torque { // FIXME unclear about correctness
            self.torque += torque
        }
    }

    public func removeFromParent() {
        guard let parentJoint = parentJoint else { return }
        let parentRigidBody = parentJoint.parentRigidBody

        self.parentJoint = nil
        parentRigidBody.childJoints.removeAll { (joint: Joint) -> Bool in
            return joint === parentJoint
        }
    }
    
    func resetForces() {
        self.force = float3.zero
        self.torque = float3.zero
    }

    // FIXME totally duplicated
    public func updateTransform() {
        guard let parentJoint = parentJoint else { return }

        let sora = parentJoint.θ[0]
        let rotation_local = simd_length(sora) < 10e-10 ? simd_quatf.identity : simd_quatf(angle: simd_length(sora), axis: normalize(sora))

        self.rotation = (parentJoint.rotation * rotation_local).normalized
        self.translation = parentJoint.translation

        self.inertiaTensor = float3x3(rotation) * inertiaTensor_local * float3x3(rotation).transpose

        self.centerOfMass = translation + rotation.act(centerOfMass_local)

        node.simdPosition = self.translation
        node.simdOrientation = self.rotation
    }
}
