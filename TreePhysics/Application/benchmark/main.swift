import Foundation
import MetalKit

let root = ArticulatedRigidBody.static()
let rigidBodyPen = RigidBodyPen(parent: root)

let rule = Rewriter.Rule(symbol: "A", replacement: #"[!"&FFFFFA&&J]////[!"&FFFFFA&&J]////[!"&FFFFFA&&J]"#)
let lSystem = Rewriter.rewrite(premise: "A", rules: [rule], generations: 5)
let configuration = InterpreterConfig(
    randomScale: 0.4,
    angle: 18 * .pi / 180,
    thickness: 0.002*0.002*Float.pi,
    thicknessScale: 0.7,
    stepSize: 0.3,
    stepSizeScale: 0.9)
let interpreter = Interpreter(configuration: configuration, pen: rigidBodyPen)
interpreter.interpret(lSystem)

let device = MTLCreateSystemDefaultDevice()!
let commandQueue = device.makeCommandQueue()!

let (rigidBodies, rigidBodiesBuffer, ranges) = UpdateCompositeBodies.rigidBodiesBuffer(root: root, device: device)
let compositeBodiesBuffer = UpdateCompositeBodies.compositeBodiesBuffer(count: rigidBodies.count, device: device)
let updateCompositeBodies = UpdateCompositeBodies(rigidBodiesBuffer: rigidBodiesBuffer, ranges: ranges, compositeBodiesBuffer: compositeBodiesBuffer)

let captureManager = MTLCaptureManager.shared()
guard captureManager.supportsDestination(.gpuTraceDocument) else
{
    fatalError()
}
let captureDescriptor = MTLCaptureDescriptor()
captureDescriptor.captureObject = device

while true {
    try! captureManager.startCapture(with: captureDescriptor)
    let commandBuffer = commandQueue.makeCommandBuffer()!
    updateCompositeBodies.encode(commandBuffer: commandBuffer)
    commandBuffer.addCompletedHandler { _ in
        print("tesssss")
    }
    commandBuffer.commit()
    commandBuffer.waitUntilCompleted()
    captureManager.stopCapture()
}
