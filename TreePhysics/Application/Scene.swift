import Foundation
import SceneKit

class Game: NSObject {
    let scene: SCNScene
    private let simulator: Simulator
    let gravityField: GravityField
    let attractorField: AttractorField
    let attractor: SCNNode
    let parent: SCNNode
    let updateCompositeBodies: UpdateCompositeBodiesKernel
    let updateJoints: UpdateJointsKernel
    let updateRigidBodies: UpdateRigidBodiesKernel
    let device: MTLDevice
    let commandQueue: MTLCommandQueue

    override init() {
        self.device = MTLCreateSystemDefaultDevice()!

        self.scene = SCNScene()

        let cameraNode = SCNNode()
        let camera = SCNCamera()
        camera.zNear = 0
        camera.zFar = 10
        cameraNode.camera = camera
        scene.rootNode.addChildNode(cameraNode)
        cameraNode.position = SCNVector3(x: 0, y: 0.75, z: 5.5)
        cameraNode.name = "Camera"

        let ambientLightNode = SCNNode()
        ambientLightNode.light = SCNLight()
        ambientLightNode.light!.type = .ambient
        ambientLightNode.name = "Ambient Light"
        scene.rootNode.addChildNode(ambientLightNode)

        // Tree:

        let root = RigidBody(length: 0, radius: 0, density: 0, kind: .static)
        let cylinderPen = CylinderPen(radialSegmentCount: 3, heightSegmentCount: 1)
        let rigidBodyPen = RigidBodyPen(parent: root)
        let skinningPen = SkinningPen(cylinderPen: cylinderPen, rigidBodyPen: rigidBodyPen)

        let rule = Rewriter.Rule(symbol: "A", replacement: #"[!"&FFFFFFA]/////[!"&FFFFFFA]/////[!"&FFFFFFA]"#)
        let lSystem = Rewriter.rewrite(premise: "A", rules: [rule], generations: 6)

        let configuration = Interpreter<SkinningPen>.Configuration(
            randomScale: 0.4,
            angle: 18 * .pi / 180,
            thickness: 0.002*0.002*Float.pi,
            thicknessScale: 0.9,
            stepSize: 0.1,
            stepSizeScale: 0.9)
        let interpreter = Interpreter(configuration: configuration, pen: skinningPen)
        interpreter.interpret(lSystem)
        let tree = Tree(root)
        self.simulator = Simulator(tree: tree)

        let geometry = cylinderPen.geometry

        var boneNodes: [SCNNode] = []
        var boneInverseBindTransforms: [NSValue] = []
        var boneWeights: [Float] = Array(repeating: 1.0, count: cylinderPen.vertices.count)
        var boneIndices: Indices = Array(repeating: 0, count: cylinderPen.vertices.count)

        self.parent = SCNNode()

        for (boneIndex, bone) in skinningPen.bones.enumerated() {
            let (vertexIndices, rigidBody) = bone
            let node = rigidBody.node
            boneNodes.append(node)
            node.isHidden = true
            parent.addChildNode(node)
            boneInverseBindTransforms.append(NSValue(scnMatrix4: SCNMatrix4Invert(node.worldTransform)))
            for vertexIndex in vertexIndices {
                boneIndices[Int(vertexIndex)] = UInt16(boneIndex)
            }
        }

        let boneWeightsData = Data(bytesNoCopy: &boneWeights, count: boneWeights.count * MemoryLayout<Float>.size, deallocator: .none)
        let boneIndicesData = Data(bytesNoCopy: &boneIndices, count: boneWeights.count * MemoryLayout<UInt16>.size, deallocator: .none)

        let boneWeightsGeometrySource = SCNGeometrySource(data: boneWeightsData, semantic: .boneWeights, vectorCount: boneWeights.count, usesFloatComponents: true, componentsPerVector: 1, bytesPerComponent: MemoryLayout<Float>.size, dataOffset: 0, dataStride: MemoryLayout<Float>.size)
        let boneIndicesGeometrySource = SCNGeometrySource(data: boneIndicesData, semantic: .boneIndices, vectorCount: boneIndices.count, usesFloatComponents: false, componentsPerVector: 1, bytesPerComponent: MemoryLayout<UInt16>.size, dataOffset: 0, dataStride: MemoryLayout<UInt16>.size)

        let skinner = SCNSkinner(baseGeometry: geometry, bones: boneNodes, boneInverseBindTransforms: boneInverseBindTransforms, boneWeights: boneWeightsGeometrySource, boneIndices: boneIndicesGeometrySource)

        let node = SCNNode(geometry: geometry)
        node.skinner = skinner

        scene.rootNode.addChildNode(node)
        scene.rootNode.addChildNode(parent)
        //        for bone in boneNodes {
        //            scene.rootNode.addChildNode(bone)
        //        }

        // Forces:
        let gravityField = GravityField(float3.zero)
        let attractorField = AttractorField()
        simulator.add(field: gravityField)
        simulator.add(field: attractorField)

        let attractor = SCNNode(geometry: SCNSphere(radius: 0.1))
        scene.rootNode.addChildNode(attractor)

        self.gravityField = gravityField
        self.attractorField = attractorField
        self.attractor = attractor

        self.commandQueue = device.makeCommandQueue()!
        let (count, rigidBodiesBuffer, ranges) = UpdateCompositeBodiesKernel.buffer(root: root, device: device)
        let compositeBodiesBuffer = device.makeBuffer(
            length: MemoryLayout<CompositeBodyStruct>.stride * count,
            options: [.storageModePrivate])!
        self.updateCompositeBodies = UpdateCompositeBodiesKernel(device: device, rigidBodiesBuffer: rigidBodiesBuffer, ranges: ranges, compositeBodiesBuffer: compositeBodiesBuffer)

        let jointsBuffer = UpdateJointsKernel.buffer(count: count, device: device)
        self.updateJoints = UpdateJointsKernel(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, numJoints: count)

        self.updateRigidBodies = UpdateRigidBodiesKernel(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, ranges: ranges)
    }
}

var start = Date()

let radius: Float = 3
extension Game: SCNSceneRendererDelegate {
    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        let pov = renderer.pointOfView!
        pov.simdPosition = float3(
            radius * sinf(Float(start.timeIntervalSinceNow)),
            1,
            radius * cosf(Float(start.timeIntervalSinceNow)))
        pov.look(at: SCNVector3(0,1,0), up: SCNVector3(0,1,0), localFront: SCNVector3(0,0,-1))
        // simulator.update(at: 1.0 / 60)
        renderer.isPlaying = true

        let commandBuffer = commandQueue.makeCommandBuffer()!
        updateCompositeBodies.encode(commandBuffer: commandBuffer)
        updateJoints.encode(commandBuffer: commandBuffer, at: 1.0 / 60)
        updateRigidBodies.encode(commandBuffer: commandBuffer)
        commandBuffer.addCompletedHandler { _ in
            print("done")
        }
        commandBuffer.commit()
    }
}
