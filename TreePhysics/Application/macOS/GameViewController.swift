import Foundation
import SceneKit
@testable import TreePhysics

class GameViewController: NSViewController {
    var root: ArticulatedRigidBody!
    var device: MTLDevice!
    var mem: MemoryLayoutManager!

    var captureScope: MTLCaptureScope!

    override func viewDidLoad() {
        super.viewDidLoad()
        
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

        self.device = SharedBuffersMTLDevice(MTLCreateSystemDefaultDevice()!)

        self.mem = MemoryLayoutManager(device: device, root: root)
        print("Total nodes:", mem.rigidBodies.count)

        scnView.delegate = self
        let scene = SCNScene()
        scene.rootNode.addChildNode(SCNNode(geometry: SCNSphere(radius: 0.1)))
        scnView.scene = scene

        scene.rootNode.addChildNode(createAxesNode(quiverLength: 1, quiverThickness: 0.25))

        scnView.allowsCameraControl = true
        scnView.backgroundColor = .gray

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
    }
}
