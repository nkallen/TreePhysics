import Foundation
import simd
import ShaderTypes

public final class WindField: PhysicsField {
    let windVelocity: float3

    public var airResistanceMultiplier: Float = 1
    public var phi: Float = .pi/4
    let leafScale: Float = 1
    let airDensity: Float = 0.1
    let normal2tangentialDragCoefficientRatio: Float = 100
    let branchScale: Float = 1

    public init(windVelocity: float3 = float3(2,0,7)) {
        self.windVelocity = windVelocity
    }

    public override func force(rigidBody: RigidBody, time: TimeInterval) -> float3 {
        switch rigidBody {
        case let internode as Internode:
            return force(internode: internode, time: time)
        // FIXME this subclass relation is lame; add some sort of mask of kind
        case let leaf as Leaf:
            return force(leaf: leaf, time: time)
        default:
            fatalError()
        }
    }

    func force(internode: Internode, time: TimeInterval) -> float3 {
        let relativeVelocity = windVelocity - internode.velocity
        let relativeVelocity_normal = dot(relativeVelocity, internode.normal) * internode.normal
        let result = branchScale * airDensity * internode.crossSectionalArea * length(relativeVelocity_normal) * relativeVelocity_normal
        return result
    }

    override func torque(rigidBody: RigidBody, time: TimeInterval) -> float3 {
        switch rigidBody {
        case _ as Internode:
            return float3.zero
        case let leaf as Leaf:
            return torque(leaf: leaf, time: time)
        default: fatalError()
        }
    }

    func force(leaf: Leaf, time: TimeInterval) -> float3 {
        let relativeVelocity: float3 = windVelocity - leaf.velocity
        let relativeVelocity_normal: float3 = dot(relativeVelocity, leaf.normal) * leaf.normal
        let relativeVelocity_tangential: float3 = relativeVelocity - relativeVelocity_normal
        let lift: float3 = leafScale * airDensity * leaf.area * length(relativeVelocity) * relativeVelocity_normal
        let k: Float = airResistanceMultiplier * leaf.mass
        let drag: float3 = k * (relativeVelocity_normal + relativeVelocity_tangential / normal2tangentialDragCoefficientRatio)
        return lift + drag
    }

    func torque(leaf: Leaf, time: TimeInterval) -> float3 {
        let relativeVelocity: float3 = windVelocity - leaf.velocity
        let k: Float = leafScale * airDensity * leaf.area / 2 * dot(relativeVelocity, leaf.normal)
        var torque: float3 = k * cross(leaf.normal, relativeVelocity * cos(phi) + cross(leaf.normal, relativeVelocity * sin(phi)))
        torque -= airResistanceMultiplier * leaf.inertiaTensor * leaf.angularVelocity
        return torque
    }
}
