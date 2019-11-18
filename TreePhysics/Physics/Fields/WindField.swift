import Foundation
import simd
import ShaderTypes

public final class WindField: PhysicsField {
    let windVelocity: SIMD3<Float>
    public var airResistanceMultiplier: Float = 4
    public var phi: Float = .pi/8
    let leafScale: Float = 1
    let airDensity: Float = 0.1
    let normal2tangentialDragCoefficientRatio: Float = 100
    let branchScale: Float = 1

    let noise = Noise()

    public init(windVelocity: SIMD3<Float> = SIMD3<Float>(2,0,7)) {
        self.windVelocity = windVelocity
    }

    override func force(rigidBody: RigidBody, time: TimeInterval) -> SIMD3<Float> {
        guard let shape = rigidBody.shape else { return .zero }

        let windVelocity = self.windVelocity(for: rigidBody, at: time)
        let relativeVelocity: SIMD3<Float> = windVelocity - rigidBody.velocity

        switch shape {
        case let .internode(area: area, length: _, radius: _):
            let normal = rigidBody.orientation.heading
            let relativeVelocity_normal = relativeVelocity - dot(relativeVelocity, normal) * normal
            let result = branchScale * airDensity * area * length(relativeVelocity_normal) * relativeVelocity_normal
            return result
        case let .leaf(area: area):
            let normal = rigidBody.orientation.up
            let relativeVelocity_normal: SIMD3<Float> = dot(relativeVelocity, normal) * normal
            let relativeVelocity_tangential: SIMD3<Float> = relativeVelocity - relativeVelocity_normal
            let lift: SIMD3<Float> = leafScale * airDensity * area * length(relativeVelocity) * relativeVelocity_normal
            let k: Float = airResistanceMultiplier * rigidBody.mass
            let drag: SIMD3<Float> = k * (relativeVelocity_normal + relativeVelocity_tangential / normal2tangentialDragCoefficientRatio)
            return lift + drag
        }
    }

    override func torque(rigidBody: RigidBody, time: TimeInterval) -> SIMD3<Float> {
        guard let shape = rigidBody.shape else { return .zero }

        switch shape {
        case .internode:
            return .zero
        case let .leaf(area: area):
            let normal = rigidBody.orientation.up

            let windVelocity = self.windVelocity(for: rigidBody, at: time)
            let relativeVelocity: SIMD3<Float> = windVelocity - rigidBody.velocity
            let k: Float = leafScale * airDensity * area / 2 * sqrt(area / .pi) * dot(relativeVelocity, normal)
            var torque: SIMD3<Float> = k * cross(normal, relativeVelocity * cos(phi) + cross(normal, relativeVelocity * sin(phi)))
            torque -= airResistanceMultiplier * rigidBody.inertiaTensor * rigidBody.angularVelocity

            return torque
        }
    }

    func windVelocity(for rigidBody: RigidBody, at time: TimeInterval) -> SIMD3<Float> {
        guard length(self.windVelocity) > 10e-10 else { return .zero }
        let r = abs(noise.fbm(rigidBody.centerOfMass.xy + Float(time), amplitude: length(self.windVelocity))) * normalize(self.windVelocity)
        return r
    }
}
