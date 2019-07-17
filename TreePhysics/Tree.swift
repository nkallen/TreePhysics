import Foundation
import SceneKit

extension Tree {
    static let K: Float = 1
    static let B: Float = 1
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
    weak var parent: Branch? {
        didSet {
            self.angle = -Float.pi / 4
        }
    }
    let name: String

    var angle: Float = 0 {
        didSet {
            node.simdRotation = float4(0, 0, 1, self.angle)
        }
    }

    init() {
        self.name = "Branch[\(i)]"
        i += 1
    }

    lazy var node: SCNNode = {
        let cylinder = SCNCylinder(radius: CGFloat(0.1), height: CGFloat(length))
        let node = SCNNode(geometry: cylinder)
        node.name = name

        node.pivot = SCNMatrix4MakeTranslation(0, CGFloat(-length / 2), 0)

        node.simdPosition = float3(0, length / 2, 0)

        return node
    }()

    func add(_ child: Branch) {
        child.parent = self
        self.children.append(child)
        self.node.addChildNode(child.node)
    }

    // on rigid body:

    let mass: Float = 1
    let length: Float = 1
    var force: float2 = float2.zero
    var torque: float3 = float3.zero
    lazy var inertia: Float = 1.0/12 * mass * length * length // Moment of Inertia of a rod about its center of mass

    // NOTE: location is along the Y axis of the cylinder/branch, relative to the pivot/parent's end
    // distance is in normalize [0..1] coordinates
    func apply(force: float2, at distance: Float) {
        precondition(distance >= 0 && distance <= 1)
        self.force += force
        self.torque += cross(force, convert(position: float2(0, 1) * distance * length) - worldPosition)
    }

    var compositeMass: Float = 0
    var compositeInertia: Float = 0
    var compositeForce: float2 = float2.zero
    var compositeTorque: float3 = float3.zero
    var compositeCenterOfMass: float2 = float2.zero

    func updateComposite() {
        for child in children {
            child.updateComposite()
        }

        self.compositeMass = mass + children.map { $0.compositeMass }.sum
        self.compositeForce = force + children.map { $0.compositeForce }.sum

        // this is due to distributivity of cross product
        self.compositeTorque = torque + children.map { child in
            return cross(child.compositeForce, child.worldPosition - self.worldPosition) + child.compositeTorque
            }.sum

        self.compositeCenterOfMass = (mass * worldCenterOfMass + children.map { $0.compositeMass * $0.compositeCenterOfMass }.sum) / (mass + children.map { $0.compositeMass }.sum)

        // using the parallel axis theorem I' = I + md^2, calculate inertia of this body about the
        // center of mass of the composite body, then add the child inertia's (also relative to the
        // center of mass of the composite)
        self.compositeInertia = inertia +
            mass * square(distance(self.worldCenterOfMass, self.compositeCenterOfMass)) +
            children.map { $0.compositeInertia + $0.compositeMass * square(distance(self.compositeCenterOfMass, $0.compositeCenterOfMass)) }.sum
    }

    var rotation: float3x3 {
        return matrix3x3_rotation(radians: angle)
    }

    private var centerOfMass: float2 {
        return float2(0, 1) * length / 2
    }

    var worldCenterOfMass: float2 {
        return convert(position: centerOfMass)
    }

    var translation: float3x3 {
        if parent != nil {
            return matrix3x3_translation(0, 1)
        } else {
            return matrix_identity_float3x3
        }
    }

    var transform: float3x3 {
        return translation * rotation
    }

    var worldTransform: float3x3 {
        if let parent = parent {
            return parent.worldTransform * transform
        } else {
            return transform
        }
    }

    var position: float2 {
        return (transform * float3(0,0,1)).xy
    }

    var worldPosition: float2 {
        return (worldTransform * float3(0,0,1)).xy
    }

    func convert(position: float2) -> float2 {
        return (worldTransform * float3(position, 1)).xy
    }

    func update(delta: TimeInterval) {
        updateComposite()
        updateSpring(delta: delta)
    }

    func updateSpring(delta: TimeInterval) {
        let compositeInertiaRelativeToJoint = compositeInertia + compositeMass * square(distance(compositeCenterOfMass, worldPosition))


        // Solve: Iθ'' + (αI + βK)θ' + Kθ = τ

        // Characteristic equation: ar^2 + br + c = 0
        let solution = solve_quadratic(a: compositeInertiaRelativeToJoint, b: Tree.BK, c: Tree.K)
    }
}

enum QuadraticSolution: Equatable {
    case real(Float)
    case realDistinct(Float, Float)
    case complex(Float, Float)
}

enum DifferentialSolution: Equatable {
    case real(c1: Float, c2: Float, r: Float)
    case realDistinct(c1: Float, c2: Float, r1: Float, r2: Float)
    case complex(c1: Float, c2: Float, lambda: Float, mu: Float)
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

func solve_differential(a: Float, b: Float, c: Float, y_0: Float, dydt_0: Float) -> DifferentialSolution {
    switch solve_quadratic(a: a, b: b, c: c) {
    case let .complex(real, imaginary):
        let c1 = y_0
        let c2 = (dydt_0 - real * c1) / imaginary
        return .complex(c1: c1, c2: c2, lambda: real, mu: imaginary)
    case let .real(r):
        let system = float2x2(columns: (float2(1, r), float2(0, 1)))
        let solution = system.inverse * float2(y_0, dydt_0)
        return .real(c1: solution.x, c2: solution.y, r: r)
    case let .realDistinct(r1, r2):
        let system = float2x2(columns: (float2(1, r1), float2(1, r2)))
        let solution = system.inverse * float2(y_0, dydt_0)
        return .realDistinct(c1: solution.x, c2: solution.y, r1: r1, r2: r2)
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
