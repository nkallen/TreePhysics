import Foundation
import SceneKit


extension Internode {
    static let K: Float = 200
    static let β: Float = 0.01
    static let maxAngle: Float = Float.pi / 3
    static let minAngle: Float = -Float.pi / 3
}

public class Internode: RigidBody {
    let area: Float
    let length: Float
    let radius: Float

    // FIXME think about these whether they should be stored properties or what
    var normal: float3 {
        return rotation.act(.y)
    }

    public init(length: Float = 1.0, radius: Float = 1.0, density: Float = 1.0/Float.pi) {

        self.area = .pi * radius * length
        self.length = length
        self.radius = radius

        let mass = .pi * sqr(radius) * length * density
        let momentOfInertiaAboutY = 1.0/12 * mass * length * length // Moment of Inertia of a rod about its center of mass
        let momentOfInertiaAboutX = 1.0/4 * mass * radius * radius // MoI of a disc about its center
        let momentOfInertiaAboutZ = 1.0/4 * mass * radius * radius // ditto

        // Inertia tensor of a rod about its center of mass, see http://scienceworld.wolfram.com/physics/MomentofInertiaCylinder.html
        // and https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        let inertiaTensor = float3x3(diagonal:
            float3(momentOfInertiaAboutY + momentOfInertiaAboutZ,
                   momentOfInertiaAboutZ + momentOfInertiaAboutX,
                   momentOfInertiaAboutX + momentOfInertiaAboutY))

        let centerOfMass = float3(0, length/2, 0)

        let node = SCNNode(geometry: SCNSphere(radius: 0.01))

        super.init(mass: mass, inertiaTensor: inertiaTensor, centerOfMass: centerOfMass, node: node)
    }

    // FIXME is this necessary?
    // NOTE: location is along the Y axis of the cylinder/branch, relative to the pivot/parent's end
    // distance is in normalize [0..1] coordinates
    func apply(force: float3, at distance: Float) {
        guard distance >= 0 && distance <= 1 else { fatalError("Force must be applied between 0 and 1") }

        let torque = cross(rotation.act(float3(0, distance * length, 0)), force)
        apply(force: force, torque: torque)
    }
}
