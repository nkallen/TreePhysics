import Foundation
import SceneKit

/**
 An `Internode` is a branch in a tree, or a segment of a branch if the branch is divided into multiple pieces. Its shape is a cylinder where (0,0,0) is the end of the cylinder where it connects to its parent `Joint`. An `Internode` always branches off from its parent joint at a rotation of (0,0,0); the rotation of the `Joint` encodes the resting state branch angle.
 */

extension Internode {
    static let K: Float = 200
    static let β: Float = 0.02
    static let maxAngle: Float = Float.pi / 3
    static let minAngle: Float = -Float.pi / 3
}

public class Internode: ArticulatedRigidBody {
    let area: Float
    let length: Float
    let radius: Float
    var axis: float3

    public init(length: Float = 1.0, radius: Float = 1.0, density: Float = 1.0/Float.pi) {
        self.area = .pi * radius * length
        self.length = length
        self.radius = radius
        self.axis = .y

        let mass = .pi * sqr(radius) * length * density
        let momentOfInertiaAboutY = 1.0/12 * mass * length * length // Moment of Inertia of a rod about its center of mass
        let momentOfInertiaAboutX = 1.0/4 * mass * radius * radius // MoI of a disc about its center
        let momentOfInertiaAboutZ = 1.0/4 * mass * radius * radius // ditto

        // Inertia tensor of a rod about its center of mass, see http://scienceworld.wolfram.com/physics/MomentofInertiaCylinder.html
        // and https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        let localInertiaTensor = float3x3(diagonal:
            float3(momentOfInertiaAboutY + momentOfInertiaAboutZ,
                   momentOfInertiaAboutZ + momentOfInertiaAboutX,
                   momentOfInertiaAboutX + momentOfInertiaAboutY))

        let localPivot = float3(0, -length/2, 0)

        let node = SCNNode(geometry: SCNSphere(radius: 0.01))

        super.init(mass: mass, localInertiaTensor: localInertiaTensor, localPivot: localPivot, node: node)
    }

    override func updateTransform() {
        super.updateTransform()
        self.axis = rotation.act(.y)
    }
}
