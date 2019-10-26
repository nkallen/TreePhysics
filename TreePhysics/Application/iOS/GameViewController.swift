import Foundation
import SceneKit
import UIKit

class GameViewController: UIViewController {
    var root: ArticulatedRigidBody!
    var device: MTLDevice!
    var argumentEncoder: UpdateCompositeBodiesArgumentEncoder!
    var ranges: [Range<Int>]!
    var compositeBodiesBuffer: MTLBuffer!

    override func viewDidLoad() {
        super.viewDidLoad()
    }

    override func viewDidAppear(_ animated: Bool) {
        self.root = ArticulatedRigidBody.static()
        let rigidBodyPen = RigidBodyPen(parent: root)

        let rule = Rewriter.Rule(symbol: "A", replacement: #"[!"&FA]////[!"&FA]////[!"&FA]"#)
        let lSystem = Rewriter.rewrite(premise: "A", rules: [rule], generations: 8)
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

        let (rigidBodies, argumentEncoder, ranges) = UpdateCompositeBodies.rigidBodiesBuffer(root: root, device: device)
        self.ranges = ranges
        self.argumentEncoder = argumentEncoder
        print(rigidBodies.count)

        self.compositeBodiesBuffer = UpdateCompositeBodies.compositeBodiesBuffer(count: rigidBodies.count, device: device)

        scnView.delegate = self
        let scene = SCNScene()
        scene.rootNode.addChildNode(SCNNode(geometry: SCNSphere(radius: 0.1)))
        scnView.scene = scene
    }

    var scnView: SCNView {
        return self.view as! SCNView
    }
}

extension GameViewController: SCNSceneRendererDelegate {
    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        let commandQueue = device.makeCommandQueue()!

        let updateCompositeBodies = UpdateCompositeBodies(argumentEncoder: argumentEncoder, ranges: ranges)

        let commandBuffer = commandQueue.makeCommandBuffer()!

        updateCompositeBodies.encode(commandBuffer: commandBuffer)

        commandBuffer.commit()
        commandBuffer.waitUntilCompleted()

        print(String.localizedStringWithFormat("%.2f ms", (commandBuffer.gpuEndTime - commandBuffer.gpuStartTime) * 1000))

    }
}
