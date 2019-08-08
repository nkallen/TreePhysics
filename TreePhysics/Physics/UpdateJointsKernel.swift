import Foundation
import MetalKit
import Metal

final class UpdateJointsKernel: MetalKernel {
    let compositeBodiesBuffer: MTLBuffer
    let jointsBuffer: MTLBuffer
    let numJoints: Int

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, compositeBodiesBuffer: MTLBuffer, jointsBuffer: MTLBuffer, numJoints: Int) {
        self.jointsBuffer = jointsBuffer
        self.compositeBodiesBuffer = compositeBodiesBuffer
        self.numJoints = numJoints
        super.init(device: device, name: "updateJoints")
    }

    func encode(commandBuffer: MTLCommandBuffer) {
        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.setComputePipelineState(computePipelineState)
        commandEncoder.label  = "Update Joints"
        commandEncoder.setBuffer(compositeBodiesBuffer, offset: 0, index: BufferIndex.compositeBodies.rawValue)
        commandEncoder.setBuffer(jointsBuffer, offset: 0, index: BufferIndex.joints.rawValue)

        let threadGroupWidth = computePipelineState.maxTotalThreadsPerThreadgroup
        let threadsPerThreadgroup = MTLSizeMake(threadGroupWidth, 1, 1)
        let threadsPerGrid = MTLSize(
            width: numJoints,
            height: 1,
            depth: 1)

        commandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        commandEncoder.endEncoding()
    }
}
