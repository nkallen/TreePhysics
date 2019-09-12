import Foundation
import simd
import SceneKit

enum Kind {
    case `static`
    case `dynamic`
}

protocol RigidBody: class {
    var kind: Kind { get } // FIXME remove

    var parentJoint: Joint? { get set }
    var composite: CompositeBody { get }

    var mass: Float { get }
    var inertiaTensor: float3x3 { get }

    var force: float3 { get }
    var torque: float3 { get }

    var centerOfMass: float3 { get }
    var rotation: simd_quatf { get }
    var angularVelocity: float3 { get }
    var angularAcceleration: float3 { get }
    var velocity: float3 { get }
    var acceleration: float3 { get }

    func apply(force: float3, torque: float3?)
    func updateTransform()
    func resetForces()

    var node: SCNNode { get }
}
