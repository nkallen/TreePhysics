import Foundation
import simd
@testable import TreePhysics

let delta: Float = 1/60

let windField = WindField()
let gravityField = GravityField(float3(0,-9.81,0))

let leafScale: Float = 1
let airDensity: Float = 0.1
let normal2tangentialDragCoefficientRatio: Float = 100
let windVelocity = float3(0,0,0)

public class Simulator {
    public var airResistanceMultiplier: Float = 3
    public var phi: Float = .pi/4

    let emitter: Emitter

    public init(emitter: Emitter) {
        self.emitter = emitter
    }

    public func update() {
        for i in 0..<emitter.count {
            let (leaf, _) = emitter.particles[i]!
            update(leaf: leaf)
        }
    }

    func update(leaf: Leaf) {
        //        gravityField.apply(rigidBody: leaf, time: Double(delta))
        //        windField.apply(rigidBody: leaf, time: Double(delta))

        let relativeVelocity: float3 = windVelocity - leaf.velocity

        let relativeVelocity_normal: float3 = dot(relativeVelocity, leaf.normal) * leaf.normal
        let relativeVelocity_tangential: float3 = relativeVelocity - relativeVelocity_normal
        let lift: float3 = leafScale * airDensity * leaf.area * length(relativeVelocity) * relativeVelocity_normal
        let drag: float3 = airResistanceMultiplier * leaf.mass * (relativeVelocity_normal + relativeVelocity_tangential / normal2tangentialDragCoefficientRatio)

        let gravity = leaf.mass * float3(0, -9.81, 0)

        let force = lift + drag + gravity

        var torque: float3 = leafScale * airDensity * leaf.area / 2 * dot(relativeVelocity, leaf.normal) * leaf.normal.crossMatrix * (relativeVelocity * cos(phi) + leaf.normal.crossMatrix * relativeVelocity * sin(phi))
        torque -= airResistanceMultiplier * leaf.inertiaTensor * leaf.angularVelocity

        leaf.acceleration = force / leaf.mass
        leaf.angularMomentum = leaf.angularMomentum + delta * torque

        leaf.velocity = leaf.velocity + delta * leaf.acceleration
        leaf.angularVelocity = leaf.inertiaTensor.inverse * leaf.angularMomentum

        leaf.translation = leaf.translation + delta * leaf.velocity
        let angularVelocityQuat = simd_quatf(real: 0, imag: leaf.angularVelocity)
        leaf.rotation = leaf.rotation + delta/2 * angularVelocityQuat * leaf.rotation
        leaf.rotation = leaf.rotation.normalized

        leaf.node.simdPosition = leaf.translation
        leaf.node.simdOrientation = leaf.rotation

        leaf.inertiaTensor = float3x3(leaf.rotation) * leaf.inertiaTensor_local * float3x3(leaf.rotation).transpose

        leaf.resetForces()
    }
}
