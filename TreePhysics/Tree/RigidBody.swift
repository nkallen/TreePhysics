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

    let mass: Float
    var inertiaTensor: float3x3
    let localInertiaTensor: float3x3
    let localCenterOfMass: float3

    var force: float3 = float3.zero
    var torque: float3 = float3.zero

    // FIXME /position/
    public var rotation: simd_quatf = simd_quatf.identity
    public var translation: float3 = float3.zero
    // FIXME is this necessary?

    var centerOfMass: float3 = float3.zero
    var angularVelocity: float3 = float3.zero
    var angularAcceleration: float3 = float3.zero
    var angularMomentum: float3 = float3.zero // FIXME acc and momentum are used Either/or
    var velocity: float3 = float3.zero
    var acceleration: float3 = float3.zero

    var node: SCNNode

    public init(mass: Float, inertiaTensor: float3x3, centerOfMass: float3, node: SCNNode) {
        self.name = "RigidBody[\(i)]"
        i += 1

        self.mass = mass
        self.localInertiaTensor = inertiaTensor
        self.inertiaTensor = localInertiaTensor
        self.localCenterOfMass = centerOfMass
        self.node = node
    }

    func apply(force: float3, torque: float3? = nil) {
        // FIXME: This torque seems wrong
        let torque = torque ?? cross(rotation.act(localCenterOfMass), force)
        self.force += force
        self.torque += torque
    }

    func resetForces() {
        self.force = float3.zero
        self.torque = float3.zero
    }
}

extension RigidBody {
    var isFinite: Bool {
        return translation.isFinite &&
            rotation.isFinite &&
            inertiaTensor.isFinite &&
            angularVelocity.isFinite &&
            angularAcceleration.isFinite &&
            velocity.isFinite &&
            acceleration.isFinite &&
            centerOfMass.isFinite
    }
}
