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
        switch rigidBody {
        case let internode as Internode:
            return force(internode: internode, time: time)
        case let leaf as Leaf:
            return force(leaf: leaf, time: time)
        default:
            return .zero
        }
    }

    func force(internode: Internode, time: TimeInterval) -> SIMD3<Float> {
        let windVelocity = self.windVelocity(for: internode, at: time)
        let relativeVelocity = windVelocity - internode.velocity
        let relativeVelocity_normal = relativeVelocity - dot(relativeVelocity, internode.axis) * internode.axis
        let result = branchScale * airDensity * internode.area * length(relativeVelocity_normal) * relativeVelocity_normal
        return result
    }

    override func torque(rigidBody: RigidBody, time: TimeInterval) -> SIMD3<Float> {
        switch rigidBody {
        case _ as Internode:
            return .zero
        case let leaf as Leaf:
            return torque(leaf: leaf, time: time)
        default:
            return .zero
        }
    }

    func force(leaf: Leaf, time: TimeInterval) -> SIMD3<Float> {
        let windVelocity = self.windVelocity(for: leaf, at: time)
        let relativeVelocity: SIMD3<Float> = windVelocity - leaf.velocity
        let relativeVelocity_normal: SIMD3<Float> = dot(relativeVelocity, leaf.normal) * leaf.normal
        let relativeVelocity_tangential: SIMD3<Float> = relativeVelocity - relativeVelocity_normal
        let lift: SIMD3<Float> = leafScale * airDensity * leaf.area * length(relativeVelocity) * relativeVelocity_normal
        let k: Float = airResistanceMultiplier * leaf.mass
        let drag: SIMD3<Float> = k * (relativeVelocity_normal + relativeVelocity_tangential / normal2tangentialDragCoefficientRatio)
        return lift + drag
    }

    func torque(leaf: Leaf, time: TimeInterval) -> SIMD3<Float> {
        let windVelocity = self.windVelocity(for: leaf, at: time)
        let relativeVelocity: SIMD3<Float> = windVelocity - leaf.velocity
        let k: Float = leafScale * airDensity * leaf.area / 2 * sqrt(leaf.area / .pi) * dot(relativeVelocity, leaf.normal)
        var torque: SIMD3<Float> = k * cross(leaf.normal, relativeVelocity * cos(phi) + cross(leaf.normal, relativeVelocity * sin(phi)))
        torque -= airResistanceMultiplier * leaf.inertiaTensor * leaf.angularVelocity

        return torque
    }

    func windVelocity(for rigidBody: RigidBody, at time: TimeInterval) -> SIMD3<Float> {
        guard length(self.windVelocity) > 10e-10 else { return .zero }
        let r = abs(noise.fbm(rigidBody.centerOfMass.xy + Float(time), amplitude: length(self.windVelocity))) * normalize(self.windVelocity)
        return r
    }
}
