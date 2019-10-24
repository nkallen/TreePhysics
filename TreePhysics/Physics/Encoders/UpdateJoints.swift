import Foundation
import MetalKit
import Metal
import ShaderTypes

final class UpdateJoints: MetalKernelEncoder {
    let rigidBodiesBuffer: MTLBuffer
    let compositeBodiesBuffer: MTLBuffer
    let jointsBuffer: MTLBuffer
    let numJoints: Int
    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, rigidBodiesBuffer: MTLBuffer, compositeBodiesBuffer: MTLBuffer, jointsBuffer: MTLBuffer, numJoints: Int) {
        precondition(numJoints > 0)

        let library = device.makeDefaultLibrary()!
        let constantValues = MTLFunctionConstantValues()
        var β = Tree.β
        constantValues.setConstantValue(&β, type: .float, index: FunctionConstantIndex.beta.rawValue)
        let function = try! library.makeFunction(name: "updateJoints", constantValues: constantValues)

        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.jointsBuffer = jointsBuffer
        self.compositeBodiesBuffer = compositeBodiesBuffer
        self.numJoints = numJoints

        super.init(device: device, function: function)
    }

    func encode(commandBuffer: MTLCommandBuffer, at time: Float) {
        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.setComputePipelineState(computePipelineState)
        commandEncoder.label  = "Update Joints"
        commandEncoder.setBuffer(rigidBodiesBuffer, offset: 0, index: BufferIndex.rigidBodies.rawValue)
        commandEncoder.setBuffer(compositeBodiesBuffer, offset: 0, index: BufferIndex.compositeBodies.rawValue)
        commandEncoder.setBuffer(jointsBuffer, offset: 0, index: BufferIndex.joints.rawValue)
        var time = time
        commandEncoder.setBytes(&time, length: MemoryLayout<Float>.size, index: BufferIndex.time.rawValue)

        let threadGroupWidth = computePipelineState.maxTotalThreadsPerThreadgroup
        let threadsPerThreadgroup = MTLSizeMake(threadGroupWidth, 1, 1)
        let numJointsExcludingRoot = numJoints - 1
        let threadsPerGrid = MTLSize(
            width: numJointsExcludingRoot,
            height: 1,
            depth: 1)

        commandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        commandEncoder.endEncoding()
    }

    static func buffer(count: Int, device: MTLDevice) -> MTLBuffer {
        let buffer = device.makeBuffer(length: count * MemoryLayout<JointStruct>.stride, options: [.storageModePrivate])!
        return buffer
    }
}
