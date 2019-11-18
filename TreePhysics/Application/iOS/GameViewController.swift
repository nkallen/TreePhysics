import Foundation
import SceneKit
import UIKit

class GameViewController: UIViewController {
    var root: ArticulatedRigidBody!
    var world: PhysicsWorld!

    var captureScope: MTLCaptureScope!
    var commandQueue: MTLCommandQueue!
    var captureManager: MTLCaptureManager!

    var device: MTLDevice!
    var mem: MemoryLayoutManager!
    var simulator: MetalSimulator!

    override func viewDidAppear(_ animated: Bool) {
        self.root = ArticulatedRigidBody.static()
        let rigidBodyPen = RigidBodyPen(parent: root)
        let cylinderPen = CylinderPen<UInt16>(radialSegmentCount: 5)
        let skinningPen = SkinningPen(cylinderPen: cylinderPen, rigidBodyPen: rigidBodyPen)

        let rule = Rewriter.Rule(symbol: "A", replacement: #"[!"&FFFFFFFA]/////[!"&FFFFFFFA]/////[!"&FFFFFFFA]"#)
        let lSystem = Rewriter.rewrite(premise: "A", rules: [rule], generations: 4)
        let configuration = InterpreterConfig(
//            randomScale: 0.4,
            angle: 18 * .pi / 180,
            thickness: 0.002*0.002*Float.pi,
            thicknessScale: 0.7,
            stepSize: 0.3,
            stepSizeScale: 0.9)
        let interpreter = Interpreter(configuration: configuration, pen: skinningPen)
        interpreter.interpret(lSystem)

        self.device = MTLCreateSystemDefaultDevice()!
        self.commandQueue = device.makeCommandQueue()!
        self.mem = MemoryLayoutManager(device: device, root: root)
        self.world = PhysicsWorld()
        self.world.add(rigidBody: root)
        self.simulator = MetalSimulator(device: device, mem: mem)
        print("Total nodes:", mem.rigidBodies.count)

//        let vectorCount = cylinderPen.branchGeometry.sources.first(where: { $0.semantic == .vertex })!.vectorCount
//        var inversePositions: [simd_float3] = []
//        var inverseOrientations: [simd_quatf] = []
//        var boneIndices: [UInt16] = Array(repeating: 0, count: vectorCount)
//
//        for (vertexIndices, rigidBody) in skinningPen.branchBones.bones {
//            inversePositions.append(-rigidBody.centerOfMass)
//            inverseOrientations.append(rigidBody.orientation.inverse)
//            let rigidBodyIndex = mem.rigidBodies.index[rigidBody]!
//            for vertexIndex in vertexIndices {
//                boneIndices[Int(vertexIndex)] = UInt16(rigidBodyIndex)
//            }
//        }

        scnView.delegate = self
        let scene = SCNScene()
        scene.rootNode.addChildNode(createAxesNode(quiverLength: 1, quiverThickness: 0.1))
        scnView.scene = scene
        scnView.allowsCameraControl = true
        scnView.backgroundColor = .gray


        scene.rootNode.addChildNode(skinningPen.node())

        scnView.scene?.rootNode.addChildNode(skinningPen.node())
        scnView.scene?.rootNode.addChildNode(skinningPen.skeleton)

//        triggerProgrammaticCapture()
    }

    func triggerProgrammaticCapture() {
        let captureManager = MTLCaptureManager.shared()
        let captureDescriptor = MTLCaptureDescriptor()
        captureDescriptor.captureObject = self.device
        do {
            try captureManager.startCapture(with: captureDescriptor)
        } catch {
            fatalError("error when trying to capture: \(error)")
        }
    }

    var scnView: SCNView {
        return self.view as! SCNView
    }
}

extension GameViewController: SCNSceneRendererDelegate {
    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        let commandQueue = device.makeCommandQueue()!
        let commandBuffer = commandQueue.makeCommandBuffer()!

        simulator.encode(commandBuffer: commandBuffer, at: 1/60)
        commandBuffer.commit()
        commandBuffer.waitUntilCompleted()
//        check()


        for rigidBody in world.rigidBodiesUnordered {
            let id = mem.rigidBodies.index[rigidBody]!
            rigidBody.node.simdPosition = simd_float3(mem.rigidBodies.pivot[id])
            rigidBody.node.simdOrientation = simd_quatf(mem.rigidBodies.orientation[id])
        }

        print(String.localizedStringWithFormat("%.2f ms", (commandBuffer.gpuEndTime - commandBuffer.gpuStartTime) * 1000), mem.rigidBodies.ranges)
    }
}

extension GameViewController {
    func check() {
        let cs = CPUSimulator(world: self.world)
        cs.updateCompositeBodies()

        self.mem.assertValid(otherwise: { id, error in
            let captureManager = MTLCaptureManager.shared()
            print(self.mem.rigidBodies.ranges)
            print(error)
            for range in mem.rigidBodies.ranges {
                if range.contains(id) {
                    print("Error in range: ", range, id - range.lowerBound)
                }
            }


//            trace(id)
//            for (i, r) in world.rigidBodiesLevelOrder.enumerated() {
//                print(i, r.composite.inertiaTensor)
//                print(i, float3x3(mem.joints.inertiaTensor[i]))
//            }
//            fatalError()
            captureManager.stopCapture()
        })
    }

    func trace(_ id: Int, indent: String = "") {
        let body = world.rigidBodiesLevelOrder[id]
        for child in body.children {
            let i = mem.rigidBodies.index[child]!
            print(indent, i, world.rigidBodiesLevelOrder[i].composite.inertiaTensor)
            print(indent, i, float3x3(mem.joints.inertiaTensor[i]))
            trace(i, indent: indent + "  ")
            print("====")
        }
    }

    func inspect(_ id: Int) {
        for range in self.mem.rigidBodies.ranges {
            if range.contains(id) {
                self.simulator.debug.print(id - range.lowerBound)
            }
        }
    }
}
