import Foundation
import MetalKit
import Metal
import ShaderTypes

final class ResetForces: MetalKernelEncoder {
    let rigidBodiesBuffer: MTLBuffer
    let numRigidBodies: Int

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, rigidBodiesBuffer: MTLBuffer, numRigidBodies: Int) {
        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.numRigidBodies = numRigidBodies

        super.init(device: device, name: "resetForces")
    }

    func encode(commandBuffer: MTLCommandBuffer) {
        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.setComputePipelineState(computePipelineState)
        commandEncoder.label  = "Reset Forces"
        commandEncoder.setBuffer(rigidBodiesBuffer, offset: 0, index: BufferIndex.rigidBodies.rawValue)

        let threadGroupWidth = computePipelineState.maxTotalThreadsPerThreadgroup
        let threadsPerThreadgroup = MTLSizeMake(threadGroupWidth, 1, 1)
        let threadsPerGrid = MTLSize(
            width: numRigidBodies,
            height: 1,
            depth: 1)

        commandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        commandEncoder.endEncoding()
    }
}
