import Foundation
import SceneKit
import simd

extension Leaf {
    static let thickness: Float = 0.001
}

final class Leaf: ArticulatedRigidBody {
    let area: Float

    // FIXME
    var normal: float3 {
        return rotation.act(float3.z)
    }

    init(length: Float = 1.0, density: Float = 1.0) {
        self.area = sqr(length)
        // Inertia tensor for rectangular plate:
        let mass = density * area * Leaf.thickness
        let inertiaTensor = float3x3(diagonal:
            float3(1/12 * mass * sqr(length),
                   1/12 * mass * sqr(length),
                   1/6  * mass * sqr(length)))

        let plane = SCNBox(width: CGFloat(length), height: CGFloat(length), length: CGFloat(0.01), chamferRadius: 0)
        plane.firstMaterial?.isDoubleSided = true
        let node = SCNNode(geometry: plane)

        super.init(
            mass: mass,
            inertiaTensor: inertiaTensor,
            centerOfMass: float3(length/2, length/2, 0),
            node: node)
    }
}
