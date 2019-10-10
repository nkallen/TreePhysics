import Foundation
import simd
import SceneKit

fileprivate var i = 0

public enum Kind {
    case `static`
    case `dynamic`
}

public class RigidBody {
    let name: String
    var kind: Kind = .dynamic

    // Invariant attributes
    let mass: Float
    let localInertiaTensor: float3x3 // relative to the center of mass

    var force: SIMD3<Float> = .zero
    var torque: SIMD3<Float> = .zero

    // State attributes that vary as a function of the simulation
    public var centerOfMass: SIMD3<Float>
    public var rotation: simd_quatf
    var velocity: SIMD3<Float>
    var acceleration: SIMD3<Float>
    var inertiaTensor: float3x3
    var angularVelocity: SIMD3<Float>
    var angularAcceleration: SIMD3<Float>
    var angularMomentum: SIMD3<Float>
    var node: SCNNode

    public init(mass: Float, localInertiaTensor: float3x3, node: SCNNode) {
        self.name = "RigidBody[\(i)]"
        i += 1

        self.mass = mass
        self.localInertiaTensor = localInertiaTensor

        self.centerOfMass = .zero
        self.rotation = simd_quatf.identity
        self.velocity = .zero
        self.acceleration = .zero
        self.angularVelocity = .zero
        self.angularAcceleration = .zero
        self.angularMomentum = .zero
        self.inertiaTensor = localInertiaTensor

        self.rotation = simd_quatf.identity

        self.node = node
    }

    func apply(force: SIMD3<Float>, torque: SIMD3<Float> = .zero) {
        self.force += force
        self.torque += torque
    }

    func resetForces() {
        self.force = .zero
        self.torque = .zero
    }

    func updateTransform() {
        node.simdPosition = self.centerOfMass
        node.simdOrientation = self.rotation
    }

    var isFinite: Bool {
        return
            rotation.isFinite &&
            inertiaTensor.isFinite &&
            angularVelocity.isFinite &&
            angularAcceleration.isFinite &&
            velocity.isFinite &&
            acceleration.isFinite &&
            centerOfMass.isFinite
    }
}
