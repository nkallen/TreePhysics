import Foundation
import simd
import SceneKit

public class Tree {
    static let leafThickness: Float = 0.001
    static let K: Float = 2000
    static let β: Float = 0.02
    static let maxAngle: Float = Float.pi / 3
    static let minAngle: Float = -Float.pi / 3

    public class func internode(length: Float = 1.0, radius: Float = 1.0, density: Float = 1.0/Float.pi) -> ArticulatedRigidBody {

        let area = .pi * radius * length

        // FIXME maybe extract inertia tensor logic into shape class?

        let mass = .pi * sqr(radius) * length * density
        let momentOfInertiaAboutY = 1.0/12 * mass * length * length // Moment of Inertia of a rod about its center of mass
        let momentOfInertiaAboutX = 1.0/4 * mass * radius * radius // MoI of a disc about its center
        let momentOfInertiaAboutZ = 1.0/4 * mass * radius * radius // ditto

        // Inertia tensor of a rod about its center of mass, see http://scienceworld.wolfram.com/physics/MomentofInertiaCylinder.html
        // and https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        let localInertiaTensor = float3x3(diagonal:
            simd_float3(momentOfInertiaAboutY + momentOfInertiaAboutZ,
                   momentOfInertiaAboutZ + momentOfInertiaAboutX,
                   momentOfInertiaAboutX + momentOfInertiaAboutY))

        let localPivot = simd_float3(0, -length/2, 0)

        let shape = Shape.internode(area: area, length: length, radius: radius)
        let node = SCNNode(geometry: SCNSphere(radius: 0.01))

        return ArticulatedRigidBody(
            kind: .dynamic,
            mass: mass,
            localInertiaTensor: localInertiaTensor,
            localPivot: localPivot,
            shape: shape,
            node: node)
    }

    public class func leaf(length: Float = 1.0, density: Float = 1.0) -> ArticulatedRigidBody {
        let area = sqr(length)

        // Inertia tensor for rectangular plate:
        let mass = density * area * Tree.leafThickness
        let localInertiaTensor = float3x3(diagonal:
            simd_float3(1/12 * mass * sqr(length),
                   1/12 * mass * sqr(length),
                   1/6  * mass * sqr(length)))

        let shape = Shape.leaf(area: area)

        let plane = SCNBox(width: CGFloat(length), height: CGFloat(length), length: CGFloat(0.01), chamferRadius: 0)
        plane.firstMaterial?.isDoubleSided = true
        let node = SCNNode(geometry: plane)

        return ArticulatedRigidBody(
            kind: .dynamic,
            mass: mass,
            localInertiaTensor: localInertiaTensor,
            localPivot: simd_float3(0, -length/2, 0),
            shape: shape,
            node: node)
    }
}
