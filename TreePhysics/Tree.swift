import Foundation
import SceneKit
import Darwin

extension Tree {
    static let K: Float = 100
    static let B: Float = 0.02
    static let BK: Float = B*K
}

class Tree {
    let root: RigidBody

    init(_ root: RigidBody) {
        self.root = root
    }

    func update(delta: TimeInterval) {
        root.update(delta: delta)
    }
}

var i = 0

class Joint: HasTransform {
    unowned let parentRigidBody: RigidBody
    let childRigidBody: RigidBody

    var transform: matrix_float3x3 = matrix_identity_float3x3
    var angularAcceleration: Float = 0
    var angularVelocity: Float = 0
    var angle: Float = 0 {
        didSet {
            node.simdRotation = float4(0, 0, 1, angle)
        }
    }

    var depth: Int {
        return parentRigidBody.depth
    }

    var bk: Float {
        return Tree.B * k
    }

    var k: Float {
        switch depth {
        case 0: return Float.infinity
        case 1: return 5 * Tree.K
        case 2: return 5 * Tree.K
        case 3: return 5 * Tree.K
        default: return Tree.K
        }
    }

    let node: SCNNode

    init(parent: RigidBody, child: RigidBody) {
        self.parentRigidBody = parent
        self.childRigidBody = child
        let node = SCNNode()
        node.addChildNode(child.node)
        node.name = "Joint"
        node.simdPosition = float3(0, parent.length/2, 0)
        self.node = node
        parent.childJoints.append(self)
        child.parentJoint = self
        updateTransform()
    }

    func updateSpringState(delta: TimeInterval) {
            let compositeInertiaRelativeToJoint = childRigidBody.composite.inertia +
                childRigidBody.composite.mass * square(distance(childRigidBody.composite.centerOfMass, self.position))

            // Solve: Iθ'' + (αI + βK)θ' + Kθ = τ
            // θ(0) = joint's angle, θ'(0) = joint's angular acceleration

            let solution = solve_differential(a: compositeInertiaRelativeToJoint, b: bk, c: k, g: childRigidBody.composite.torque.z, y_0: angle, y_ddt_0: angularVelocity)
            let thetas = evaluate(differential: solution, at: Float(delta))
            self.angle = thetas.x
            self.angularVelocity = thetas.y
            self.angularAcceleration = thetas.z

        for joint in childRigidBody.childJoints {
            joint.updateSpringState(delta: delta)
        }
    }

    func updateTransform() {
        self.transform = parentRigidBody.transform * matrix3x3_translation(0, parentRigidBody.length/2)
    }
}

class RigidBody: HasTransform {
    let name: String
    weak var parentJoint: Joint?
    var childJoints: [Joint] = []

    var composite: CompositeBody!

    let mass: Float
    let length: Float
    let inertia: Float

    var transform: matrix_float3x3 = matrix_identity_float3x3
    var force: float2 = float2.zero
    var torque: float3 = float3.zero

    let node: SCNNode

    // NOTE: the branch, in its resting state, might sprout off its parent at an angle. Thus, the
    // joint angle (which is 0 in the resting state) is not the same as the branch angle.
    var angle: Float = 0 {
        didSet {
            guard let parentJoint = parentJoint else { fatalError("A root cannot have an angle") }
            self.transform = parentJoint.transform * matrix3x3_rotation(radians: self.angle)
            node.simdRotation = float4(0, 0, 1, self.angle)
        }
    }

    var depth: Int {
        if let parent = parentJoint {
            return parent.depth + 1
        } else {
            return 0
        }
    }

    init(mass: Float = 1.0, length: Float = 1.0) {
        self.name = "Branch[\(i)]"
        i += 1

        self.mass = mass
        self.length = length
        self.inertia = 1.0/12 * mass * length * length // Moment of Inertia of a rod about its center of mass

        let cylinder = SCNCylinder(radius: CGFloat(0.01), height: CGFloat(length))
        let node = SCNNode(geometry: cylinder)
        node.name = name
        node.pivot = SCNMatrix4MakeTranslation(0, CGFloat(-length / 2), 0)
        self.node = node

        self.composite = CompositeBody(parent: self)
    }

    func add(_ child: RigidBody, at angle: Float = -Float.pi / 4) {
        let joint = Joint(parent: self, child: child)
        child.angle = angle
        self.node.addChildNode(joint.node)
    }

    // NOTE: location is along the Y axis of the cylinder/branch, relative to the pivot/parent's end
    // distance is in normalize [0..1] coordinates
    func apply(force: float2, at distance: Float) {
        guard distance >= 0 && distance <= 1 else { fatalError("Force must be applied between 0 and 1") }

        self.force += force
        self.torque += cross(convert(position: float2(0, 1) * distance * length) - self.position, force)
    }

    lazy var centerOfMass: float2 = {
        let localCenterOfMass = float2(0, 1) * length / 2
        return convert(position: localCenterOfMass)
    }()

    func update(delta: TimeInterval) {
        composite.update()
        for joint in childJoints {
            joint.updateSpringState(delta: delta)
        }
        updateRigidBodyState()
        reset()
    }

    func reset() {
        self.force = float2.zero
        self.torque = float3.zero
        for joint in childJoints {
            joint.childRigidBody.reset()
        }
    }

    private func updateRigidBodyState() {
        if let parent = parentJoint {
            self.transform = parent.transform
        } else {
            self.transform = matrix_identity_float3x3
        }

        let localCenterOfMass = float2(0, 1) * length / 2
        self.centerOfMass = convert(position: localCenterOfMass)

        for joint in childJoints {
            joint.updateTransform()
            joint.childRigidBody.updateRigidBodyState()
        }
    }
}

class CompositeBody {
    unowned let parentRigidBody: RigidBody

    var mass: Float = 0
    var inertia: Float = 0
    var force: float2 = float2.zero
    var torque: float3 = float3.zero
    var centerOfMass: float2 = float2.zero

    init(parent: RigidBody) {
        self.parentRigidBody = parent
    }

    func update() {
        for joint in parentRigidBody.childJoints {
            joint.childRigidBody.composite.update()
        }

        self.mass = parentRigidBody.mass + parentRigidBody.childJoints.map { $0.childRigidBody.composite.mass }.sum
        self.force = parentRigidBody.force + parentRigidBody.childJoints.map { $0.childRigidBody.composite.force }.sum

        // this is due to distributivity of cross product
        self.torque = parentRigidBody.torque + parentRigidBody.childJoints.map {
            cross($0.position - parentRigidBody.position, $0.childRigidBody.composite.force) + $0.childRigidBody.composite.torque
            }.sum

        self.centerOfMass = (parentRigidBody.mass * parentRigidBody.centerOfMass + parentRigidBody.childJoints.map { $0.childRigidBody.composite.mass * $0.childRigidBody.composite.centerOfMass }.sum) / (parentRigidBody.mass + parentRigidBody.childJoints.map { $0.childRigidBody.composite.mass }.sum)

        // using the parallel axis theorem I' = I + md^2, calculate inertia of this body about the
        // center of mass of the composite body, then add the child inertia's (also relative to the
        // center of mass of the composite)
        self.inertia = parentRigidBody.inertia +
            parentRigidBody.mass * square(distance(parentRigidBody.centerOfMass, self.centerOfMass)) +
            parentRigidBody.childJoints.map { $0.childRigidBody.composite.inertia + $0.childRigidBody.composite.mass * square(distance(self.centerOfMass, $0.childRigidBody.composite.centerOfMass)) }.sum
    }
}
