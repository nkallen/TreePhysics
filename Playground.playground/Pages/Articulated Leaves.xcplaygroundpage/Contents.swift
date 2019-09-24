import AppKit
import SceneKit
import PlaygroundSupport
@testable import TreePhysics

let delta: Double = 1/60

public class Scene: SCNScene, SCNSceneRendererDelegate {
    let simulator: CPUSimulator

    public init(simulator: CPUSimulator) {
        self.simulator = simulator
        super.init()
    }

    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }

    public func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        simulator.update(at: delta)
    }
}

public class MyField: PhysicsField {
    public var position = float3.zero
    public var halfExtent: float3? = nil

    public var airResistanceMultiplier: Float = 1
    public var phi: Float = .pi/4
    let leafScale: Float = 1
    let airDensity: Float = 0.1
    let normal2tangentialDragCoefficientRatio: Float = 100
    let branchScale: Float = 1

    public func force(rigidBody: RigidBody, time: TimeInterval) -> float3 {
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
        let windVelocity = float3(0,1,1) * 5
        let relativeVelocity = windVelocity - internode.velocity
        let relativeVelocity_normal = dot(relativeVelocity, internode.normal) * internode.normal
        let result = branchScale * airDensity * internode.crossSectionalArea * length(relativeVelocity_normal) * relativeVelocity_normal
        return result
    }

    public func torque(rigidBody: RigidBody, time: TimeInterval) -> float3? {
        switch rigidBody {
        case let _ as Internode:
            return nil
        case let leaf as Leaf:
            return torque(leaf: leaf, time: time)
        default: fatalError()
        }
    }

    let start = Date()
    func force(leaf: Leaf, time: TimeInterval) -> float3 {
        let windVelocity = float3(0,1,1) * 5
        let relativeVelocity: float3 = windVelocity - leaf.velocity
        let relativeVelocity_normal: float3 = dot(relativeVelocity, leaf.normal) * leaf.normal
        let relativeVelocity_tangential: float3 = relativeVelocity - relativeVelocity_normal
        let lift: float3 = leafScale * airDensity * leaf.area * length(relativeVelocity) * relativeVelocity_normal
        let drag: float3 = airResistanceMultiplier * leaf.mass * (relativeVelocity_normal + relativeVelocity_tangential / normal2tangentialDragCoefficientRatio)
        return lift + drag
    }

    func torque(leaf: Leaf, time: TimeInterval) -> float3 {
        let windVelocity = float3(0,1,1) * 10
        let relativeVelocity: float3 = windVelocity - leaf.velocity
        let relativeVelocity_normal: float3 = dot(relativeVelocity, leaf.normal) * leaf.normal
        let relativeVelocity_tangential: float3 = relativeVelocity - relativeVelocity_normal
        var torque: float3 = leafScale * airDensity * leaf.area / 2 * dot(relativeVelocity, leaf.normal) * leaf.normal.crossMatrix * (relativeVelocity * cos(phi) + leaf.normal.crossMatrix * relativeVelocity * sin(phi))
        torque -= airResistanceMultiplier * leaf.inertiaTensor * leaf.angularVelocity
        return torque
    }
}

let root = Internode(length: 0, radius: 0, density: 0, kind: .static)
let cylinderPen = CylinderPen(radialSegmentCount: 3, heightSegmentCount: 1)
let rigidBodyPen = RigidBodyPen(parent: root)
let skinningPen = SkinningPen(cylinderPen: cylinderPen, rigidBodyPen: rigidBodyPen)
let configuration = Interpreter<SkinningPen>.Configuration(
    angle: .pi / 10,
    thickness: 0.002*0.002*Float.pi,
    stepSize: 0.4)
let interpreter = Interpreter(configuration: configuration, pen: skinningPen)
interpreter.interpret("F+F+F+F+F+F+F+F+J")

let windField = MyField()

let simulator = CPUSimulator(root: root)
simulator.add(field: windField)
let scene = Scene(simulator: simulator)

let view = SCNView(frame: CGRect(x:0 , y:0, width: 640, height: 480))

let cameraNode = SCNNode()
let camera = SCNCamera()
cameraNode.camera = camera
camera.zNear = 0
camera.zFar = 10

cameraNode.camera = camera
scene.rootNode.addChildNode(cameraNode)
cameraNode.position = SCNVector3(x: -1, y: 0.75, z: 4)

scene.rootNode.addChildNode(skinningPen.node)
scene.rootNode.addChildNode(cameraNode)

view.delegate = scene
view.scene = scene
view.backgroundColor = .black
view.showsStatistics = true
view.allowsCameraControl = true
view.isPlaying = true

PlaygroundPage.current.liveView = view
