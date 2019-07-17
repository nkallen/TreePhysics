import Foundation
import SceneKit
import Darwin

extension Tree {
    static let K: Float = 100
    static let B: Float = 0.02
    static let BK: Float = B*K
}

class Tree {
    let root: Branch

    init(_ root: Branch) {
        self.root = root
    }

    func update(delta: TimeInterval) {
        root.update(delta: delta)
    }
}

var i = 0

class Branch {
    var children: [Branch] = []
    weak var parent: Branch?
    let name: String

    // NOTE: the branch, in its resting state, might sprout off its parent at an angle. Thus, the
    // joint angle (which is 0 in the resting state) is not the same as the branch angle.

    var branchAngle: Float = 0 {
        didSet {
            node.simdRotation = float4(0, 0, 1, self.netAngle)
        }
    }

    var jointAngle: Float = 0 {
        didSet {
            print("\(name): setting joint angle to", self.jointAngle, "with net angle", self.netAngle)
            node.simdRotation = float4(0, 0, 1, self.netAngle)
        }
    }

    var jointAngularAcceleration: Float = 0
    var jointAngularVelocity: Float = 0

    var netAngle: Float {
        return self.jointAngle + self.branchAngle
    }

    init() {
        self.name = "Branch[\(i)]"
        i += 1
    }

    lazy var node: SCNNode = {
        let cylinder = SCNCylinder(radius: CGFloat(0.01), height: CGFloat(length))
        let node = SCNNode(geometry: cylinder)
        node.name = name

        node.pivot = SCNMatrix4MakeTranslation(0, CGFloat(-length / 2), 0)

        node.simdPosition = float3(0, length / 2, 0)

        return node
    }()

    func add(_ child: Branch, at angle: Float = -Float.pi / 8) {
        child.parent = self
        child.branchAngle = angle
        self.children.append(child)
        self.node.addChildNode(child.node)
    }

    // on rigid body:

    var depth: Int {
        if let parent = parent {
            return parent.depth + 1
        } else {
            return 0
        }
    }

    var mass: Float {
        return 1.0 / Float(depth)
    }
    let length: Float = 0.1
    var force: float2 = float2.zero
    var torque: float3 = float3.zero
    lazy var inertia: Float = 1.0/12 * mass * length * length // Moment of Inertia of a rod about its center of mass

    // NOTE: location is along the Y axis of the cylinder/branch, relative to the pivot/parent's end
    // distance is in normalize [0..1] coordinates
    func apply(force: float2, at distance: Float) {
        precondition(distance >= 0 && distance <= 1)
        self.force += force
        self.torque += cross(force, convert(position: float2(0, 1) * distance * length) - jointPosition)
    }

    var compositeMass: Float = 0
    var compositeInertia: Float = 0
    var compositeForce: float2 = float2.zero
    var compositeTorque: float3 = float3.zero
    var compositeCenterOfMass: float2 = float2.zero

    func updateCompositeBodyState() {
        for child in children {
            child.updateCompositeBodyState()
        }

        self.compositeMass = mass + children.map { $0.compositeMass }.sum
        self.compositeForce = force + children.map { $0.compositeForce }.sum

        // this is due to distributivity of cross product
        self.compositeTorque = torque + children.map { child in
            return cross(child.compositeForce, child.jointPosition - self.jointPosition) + child.compositeTorque
            }.sum

        self.compositeCenterOfMass = (mass * worldCenterOfMass + children.map { $0.compositeMass * $0.compositeCenterOfMass }.sum) / (mass + children.map { $0.compositeMass }.sum)

        // using the parallel axis theorem I' = I + md^2, calculate inertia of this body about the
        // center of mass of the composite body, then add the child inertia's (also relative to the
        // center of mass of the composite)
        self.compositeInertia = inertia +
            mass * square(distance(self.worldCenterOfMass, self.compositeCenterOfMass)) +
            children.map { $0.compositeInertia + $0.compositeMass * square(distance(self.compositeCenterOfMass, $0.compositeCenterOfMass)) }.sum
    }


    // FIXME really we should have three objects, including the joint, and our position is (0,0) relative to the joint
    lazy var translation: float3x3 = {
        if parent != nil {
            return matrix3x3_translation(0, 1)
        } else {
            return matrix_identity_float3x3
        }
    }()

    var transform: float3x3 {
        return translation * rotation
    }

    lazy var rotation: float3x3 = matrix3x3_rotation(radians: self.branchAngle)
    lazy var worldTransform: float3x3 = {
        if let parent = parent {
            return parent.worldTransform * transform
        } else {
            return transform
        }
    }()
    lazy var jointPosition: float2 = convert(position: float2.zero)

    lazy var worldCenterOfMass: float2 = {
        let localCenterOfMass = float2(0, 1) * length / 2
        return convert(position: localCenterOfMass)
    }()

    func convert(position: float2) -> float2 {
        return (worldTransform * float3(position, 1)).xy
    }

    func update(delta: TimeInterval) {
        updateCompositeBodyState()
        updateSpringState(delta: delta)
        updateRigidBodyState()
        reset()
    }

    func reset() {
        self.force = float2.zero
        self.torque = float3.zero
        for child in children {
            child.reset()
        }
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

    func updateSpringState(delta: TimeInterval) {
        print("\(name) stiffness:", k)
        if parent != nil {
            let compositeInertiaRelativeToJoint = compositeInertia + compositeMass * square(distance(compositeCenterOfMass, jointPosition))

            // Solve: Iθ'' + (αI + βK)θ' + Kθ = τ
            // θ(0) = joint's angle, θ'(0) = joint's angular acceleration

            let solution = solve_differential(a: compositeInertiaRelativeToJoint, b: bk, c: k, g: compositeTorque.z, y_0: jointAngle, y_ddt_0: jointAngularVelocity)
            let thetas = evaluate(differential: solution, at: Float(delta))
            self.jointAngle = thetas.x
            self.jointAngularVelocity = thetas.y
            self.jointAngularAcceleration = thetas.z
        }

        for child in children {
            child.updateSpringState(delta: delta)
        }
    }

    func updateRigidBodyState() {
        // translation is in local coordinates and never changes. But, we need to
        // update rotation, worldTransform, jointPosition, worldCenterOfMass
        self.rotation = matrix3x3_rotation(radians: self.netAngle)
        if let parent = parent {
            self.worldTransform = parent.worldTransform * transform
        } else {
            self.worldTransform = transform
        }
        self.jointPosition = convert(position: float2.zero)

        let localCenterOfMass = float2(0, 1) * length / 2
        self.worldCenterOfMass = convert(position: localCenterOfMass)

        for child in children {
            child.updateRigidBodyState()
        }
    }
}

enum QuadraticSolution: Equatable {
    case real(Float)
    case realDistinct(Float, Float)
    case complex(Float, Float)
}

enum DifferentialSolution: Equatable {
    case real(c1: Float, c2: Float, r: Float, k: Float)
    case realDistinct(c1: Float, c2: Float, r1: Float, r2: Float, k: Float)
    case complex(c1: Float, c2: Float, lambda: Float, mu: Float, k: Float)
}

func solve_quadratic(a: Float, b: Float, c: Float) -> QuadraticSolution {
    //    (-b +/- sqrt(b^2 - 4ac)) / 2a
    let b2_4ac = b*b - 4*a*c
    let _2a = 1.0 / (2*a)
    if b2_4ac == 0 {
        let b_2a = -b * _2a
        return .real(b_2a)
    } else if b2_4ac > 0 {
        let b_2a = -b * _2a
        let sqrt_b2_4ac_2a = sqrt(b2_4ac) * _2a
        return .realDistinct(b_2a + sqrt_b2_4ac_2a, b_2a - sqrt_b2_4ac_2a)
    } else {
        let imaginaryPart = sqrt(-b2_4ac) * _2a
        let realPart = -b * _2a
        return .complex(realPart, imaginaryPart)
    }
}

func solve_differential(a: Float, b: Float, c: Float, g: Float, y_0: Float, y_ddt_0: Float) -> DifferentialSolution {
    switch solve_quadratic(a: a, b: b, c: c) {
    case let .complex(real, imaginary):
        let c1 = y_0
        let c2 = (y_ddt_0 - real * c1) / imaginary
        return .complex(c1: c1, c2: c2, lambda: real, mu: imaginary, k: g/c)
    case let .real(r):
        let system = float2x2(columns: (float2(1, r), float2(0, 1)))
        let solution = system.inverse * float2(y_0, y_ddt_0)
        return .real(c1: solution.x, c2: solution.y, r: r, k: g/c)
    case let .realDistinct(r1, r2):
        let system = float2x2(columns: (float2(1, r1), float2(1, r2)))
        let solution = system.inverse * float2(y_0, y_ddt_0)
        return .realDistinct(c1: solution.x, c2: solution.y, r1: r1, r2: r2, k: g/c)
    }
}

// Evaluate 2nd-order differential equation given its analytic solution
func evaluate(differential: DifferentialSolution, at t: Float) -> float3 {
    switch differential {
    case let .complex(c1: c1, c2: c2, lambda: lambda, mu: mu, k: k):
        let y = c1*powf(.e,lambda*t)*cos(mu*t) + c2*powf(.e,lambda*t)*sin(mu*t) + k
        let y_ddt = lambda*c1*powf(.e,lambda*t)*cos(mu*t) - mu*c1*powf(.e,lambda*t)*sin(mu*t) +
            lambda*c2*powf(.e,lambda*t)*sin(mu*t) + mu*c2*powf(.e,lambda*t)*cos(mu*t)
        let y_d2dt = lambda*lambda*c1*powf(.e,lambda*t)*cos(mu*t) - mu*lambda*c1*powf(.e,lambda*t)*sin(mu*t) -
            (lambda*mu*c1*powf(.e,lambda*t)*sin(mu*t) + mu*mu*c1*powf(.e,lambda*t)*cos(mu*t)) +
            lambda*lambda*c2*powf(.e,lambda*t)*sin(mu*t) + mu*lambda*c2*powf(.e,lambda*t)*cos(mu*t) +
            lambda*mu*c2*powf(.e,lambda*t)*cos(mu*t) - mu*mu*c2*powf(.e,lambda*t)*sin(mu*t)
        return float3(y, y_ddt, y_d2dt)
    case let .real(c1: c1, c2: c2, r: r, k: k):
        let y = c1*powf(.e,r*t) + c2*t*powf(.e,r*t) + k
        let y_ddt = r*c1*powf(.e,r*t) +
            c2*powf(.e,r*t) + r*c2*t*powf(.e,r*t)
        let y_d2dt = r*r*c1*powf(.e,r*t) +
            r*c2*powf(.e,r*t) +
            r*c2*powf(.e,r*t) + r*r*c2*t*powf(.e,r*t)
        return float3(y, y_ddt, y_d2dt)
    case let .realDistinct(c1: c1, c2: c2, r1: r1, r2: r2, k: k):
        let y = c1*powf(.e,r1*t) + c2*powf(.e,r2*t) + k
        let y_ddt = r1*c1*powf(.e,r1*t) +
            r2*c2 * powf(.e,r2*t)
        let y_d2dt = r1*r1*c1 * powf(.e,r1*t) +
            r2*r2*c2 * powf(.e,r2*t)
        return float3(y, y_ddt, y_d2dt)
    }
}

extension Array where Element == Float {
    var sum: Float {
        return reduce(0, +)
    }
}

extension Array where Element == float2 {
    var sum: float2 {
        return reduce(float2.zero, +)
    }
}

extension Array where Element == float3 {
    var sum: float3 {
        return reduce(float3.zero, +)
    }
}

func matrix3x3_rotation(radians: Float) -> float3x3 {
    let cs = cosf(radians)
    let sn = sinf(radians)
    return matrix_float3x3.init(columns:
        (float3(cs, sn, 0),
         float3(-sn, cs, 0),
         float3(0, 0, 1)))
}

func matrix3x3_translation(_ translationX: Float, _ translationY: Float) -> float3x3 {
    return matrix_float3x3.init(columns:(vector_float3(1, 0, 0),
                                         vector_float3(0, 1, 0),
                                         vector_float3(translationX, translationY, 1)))
}

extension float3 {
    init(_ float2: float2, _ z: Float) {
        self = float3(float2.x, float2.y, z)
    }

    var xy: float2 {
        return float2(x, y)
    }
}

func square(_ x: Float) -> Float {
    return x * x
}

extension Float {
    static let e: Float = Float(Darwin.M_E)
}
