import Foundation
import SceneKit
import simd

/**
 A `Leaf` represents a physical leaf in a physical tree (i.e., the plant) as opposed to a node in a datastructure with no children. It's shape is a very thin flat plate where (0,0,0) is the stem-end of the leaf. When interacting with the wind it is subject to lift forces and so on.
 */
extension Leaf {
    static let thickness: Float = 0.001
}

public final class Leaf: ArticulatedRigidBody {
    let area: Float
    var normal: SIMD3<Float>
    public init(length: Float = 1.0, density: Float = 1.0) {
        self.area = sqr(length)
        self.normal = .z

        // Inertia tensor for rectangular plate:
        let mass = density * area * Leaf.thickness
        let localInertiaTensor = float3x3(diagonal:
            SIMD3<Float>(1/12 * mass * sqr(length),
                   1/12 * mass * sqr(length),
                   1/6  * mass * sqr(length)))

        let plane = SCNBox(width: CGFloat(length), height: CGFloat(length), length: CGFloat(0.01), chamferRadius: 0)
        plane.firstMaterial?.isDoubleSided = true
        let node = SCNNode(geometry: plane)

        super.init(
            mass: mass,
            localInertiaTensor: localInertiaTensor,
            localPivot: SIMD3<Float>(0, -length/2, 0),
            node: node)
    }

    override func updateTransform() {
        super.updateTransform()
        self.normal = rotation.act(.z)
    }
}
