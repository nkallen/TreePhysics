import Foundation
import MetalKit
import Metal
import ShaderTypes

final class ApplyPhysicsFields: MetalKernelEncoder {
    let rigidBodiesBuffer: MTLBuffer
    let numRigidBodies: Int

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, rigidBodiesBuffer: MTLBuffer, numRigidBodies: Int) {
        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.numRigidBodies = numRigidBodies

        super.init(device: device, name: "applyPhysicsFields")
    }

    func encode(commandBuffer: MTLCommandBuffer, field: PhysicsFieldStructConvertible) {
        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.setComputePipelineState(computePipelineState)
        commandEncoder.label  = "Apply Physics Fields"
        commandEncoder.setBuffer(rigidBodiesBuffer, offset: 0, index: BufferIndex.rigidBodies.rawValue)
        var `struct` = field.struct
        commandEncoder.setBytes(&`struct`, length: MemoryLayout<PhysicsFieldStruct>.stride, index: BufferIndex.physicsField.rawValue)

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
