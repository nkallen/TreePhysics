import Foundation
import SceneKit
import UIKit

class GameViewController: UIViewController {
    var root: ArticulatedRigidBody!
    var device: MTLDevice!
    var mem: MemoryLayoutManager!

    var captureScope: MTLCaptureScope!

    override func viewDidAppear(_ animated: Bool) {
        self.root = ArticulatedRigidBody.static()
        let rigidBodyPen = RigidBodyPen(parent: root)

        let rule = Rewriter.Rule(symbol: "A", replacement: #"[!"&FA]////[!"&FA]////[!"&FA]"#)
        let lSystem = Rewriter.rewrite(premise: "A", rules: [rule], generations: 1)
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

        //        let root2 = ArticulatedRigidBody.static()
        //        let b0 = Tree.internode(length: 1, radius: 1)
        //        let b1 = Tree.internode(length: 1, radius: 1)
        //        let b0joint = root2.add(b0, rotation: .identity, position: .zero)
        //        b0joint.stiffness = 1
        //        b0joint.torqueThreshold = .infinity
        //        b0joint.damping = 1
        //
        //        let b1Joint = b0.add(b1, rotation: simd_quatf(angle: -.pi/4, axis: .z), position: simd_float3(0,1,0))
        //        b1Joint.stiffness = 1
        //        b1Joint.torqueThreshold = .infinity
        //        b1Joint.damping = 1
        //        let force = simd_float3(1, 0, 0) // world coordinates
        //
        //        b1.apply(force: force)

        self.mem = MemoryLayoutManager(device: SharedBuffersMTLDevice(device), root: root)
        print("Total nodes:", mem.rigidBodies.count)

        scnView.delegate = self
        let scene = SCNScene()
        scene.rootNode.addChildNode(SCNNode(geometry: SCNSphere(radius: 0.1)))
        scnView.scene = scene

        self.captureScope = MTLCaptureManager.shared().makeCaptureScope(device: MTLCreateSystemDefaultDevice()!)
        triggerProgrammaticCaptureScope()
    }

    var scnView: SCNView {
        return self.view as! SCNView
    }

    func triggerProgrammaticCaptureScope() {
        print("triggerProgrammaticCaptureScope")
        let captureManager = MTLCaptureManager.shared()
        let captureDescriptor = MTLCaptureDescriptor()
        captureDescriptor.captureObject = captureScope
        do {
            try captureManager.startCapture(with:captureDescriptor)
        } catch {
            fatalError("error when trying to capture: \(error)")
        }
    }
}

extension GameViewController: SCNSceneRendererDelegate {
    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        let commandQueue = device.makeCommandQueue()!

        print("beginning capture scope")
        captureScope.begin()
        let commandBuffer = commandQueue.makeCommandBuffer()!

        UpdateCompositeBodies(memoryLayoutManager: mem).encode(commandBuffer: commandBuffer)
        UpdateJoints(memoryLayoutManager: mem).encode(commandBuffer: commandBuffer, at: 1/60)
        UpdateRigidBodies(memoryLayoutManager: mem).encode(commandBuffer: commandBuffer)
        commandBuffer.commit()
        commandBuffer.waitUntilCompleted()

        print(String.localizedStringWithFormat("%.2f ms", (commandBuffer.gpuEndTime - commandBuffer.gpuStartTime) * 1000), mem.rigidBodies.ranges)

        for id in 1..<4 {
            print("id: \(id), theta: \(mem.joints[id][0])")
            print("id: \(id), x: \(mem.rigidBodies[id])")

            if !mem.joints[id].isFinite || !mem.rigidBodies[id].isValid {
                self.captureScope.end()
            }
        }
    }
}
