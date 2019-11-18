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

        let rule = Rewriter.Rule(symbol: "A", replacement: #"[!"&FFFFFFFA]////[!"&FFFFFFFA]////[!"&FFFFFFFA]"#)
        let lSystem = Rewriter.rewrite(premise: "A", rules: [rule], generations: 3)
        let configuration = InterpreterConfig(
            randomScale: 0.4,
            angle: 18 * .pi / 180,
            thickness: 0.002*0.002*Float.pi,
            thicknessScale: 0.7,
            stepSize: 0.3,
            stepSizeScale: 0.9)
        let interpreter = Interpreter(configuration: configuration, pen: rigidBodyPen)
        interpreter.interpret(lSystem)

        self.device = MTLCreateSystemDefaultDevice()!
        self.commandQueue = device.makeCommandQueue()!
        self.mem = MemoryLayoutManager(device: device, root: root)
        self.world = PhysicsWorld()
        self.world.add(rigidBody: root)
        self.simulator = MetalSimulator(device: device, mem: mem)
        print("Total nodes:", mem.rigidBodies.count)

        scnView.delegate = self
        let scene = SCNScene()
        scene.rootNode.addChildNode(createAxesNode(quiverLength: 1, quiverThickness: 0.1))
        scnView.scene = scene
        scnView.allowsCameraControl = true
        scnView.backgroundColor = .gray

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
        check()

        print(String.localizedStringWithFormat("%.2f ms", (commandBuffer.gpuEndTime - commandBuffer.gpuStartTime) * 1000), mem.rigidBodies.ranges)
    }
}

extension GameViewController {
    func check() {
        let cs = CPUSimulator(world: self.world)
        cs.updateCompositeBodies()

        self.mem.assertValid(otherwise: { error in
            //            let captureManager = MTLCaptureManager.shared()
            for (i, r) in world.rigidBodiesLevelOrder.enumerated() {
                print(i, r.composite.inertiaTensor)
                print(i, float3x3(mem.joints.inertiaTensor[i]))
            }
            fatalError()
            //            captureManager.stopCapture()
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

        //        print("=============== \(id) -- \(self.mem.rigidBodies.index[self.world.rigidBodiesLevelOrder[id]])")
        //        print(self.world.rigidBodiesLevelOrder[id].inertiaTensor)
        //        print(float3x3(self.mem.rigidBodies.inertiaTensor[id]))
        //
        //        print(self.world.rigidBodiesLevelOrder[id].composite.inertiaTensor)
        //        print(float3x3(self.mem.compositeBodies.inertiaTensor[id]))
        //
        //        print(self.world.rigidBodiesLevelOrder[id].composite.inertiaTensor.determinant)
        //        print(float3x3(self.mem.compositeBodies.inertiaTensor[id]).determinant)
        //
        //        print(self.world.rigidBodiesLevelOrder[id].composite.inertiaTensor.cholesky)
        //        print(float3x3(self.mem.compositeBodies.inertiaTensor[id]).cholesky)
        //
        //        print(self.world.rigidBodiesLevelOrder[id].composite.centerOfMass)
        //        print(float3(self.mem.compositeBodies.centerOfMass[id]))
        //
        //        for child in self.world.rigidBodiesLevelOrder[id].children {
        //            print(child.name, self.mem.rigidBodies.index[child])
        //        }
    }
}
