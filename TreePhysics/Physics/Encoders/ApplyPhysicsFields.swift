import Foundation
import MetalKit
import Metal
import ShaderTypes

final class ApplyPhysicsFields: MetalKernelEncoder {
    private let argumentEncoder: ArgumentEncoder

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, memoryLayoutManager: MemoryLayoutManager) {
        let library = device.makeDefaultLibrary()!
        let constantValues = MTLFunctionConstantValues()
        var fieldCount = memoryLayoutManager.physicsFields.count
        constantValues.setConstantValue(&fieldCount, type: .int, index: FunctionConstantIndex.physicsFieldCount.rawValue)
        let function = try! library.makeFunction(name: "applyPhysicsFields", constantValues: constantValues)

        self.argumentEncoder = ArgumentEncoder(memoryLayoutManager: memoryLayoutManager)

        super.init(device: device, function: function)
    }

    func encode(commandBuffer: MTLCommandBuffer, at time: Float) {
        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.setComputePipelineState(computePipelineState)
        commandEncoder.label  = "Apply Physics Fields"
        argumentEncoder.encode(commandEncoder: commandEncoder, at: time)

        let threadGroupWidth = computePipelineState.maxTotalThreadsPerThreadgroup
        let threadsPerThreadgroup = MTLSizeMake(threadGroupWidth, 1, 1)
        let count = argumentEncoder.mem.rigidBodies.count
        let threadsPerGrid = MTLSize(
            width: count,
            height: 1,
            depth: 1)

        commandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        commandEncoder.endEncoding()
    }
}

extension ApplyPhysicsFields {
    class ArgumentEncoder {
        let mem: MemoryLayoutManager

        init(memoryLayoutManager: MemoryLayoutManager) {
            self.mem = memoryLayoutManager
        }

        func encode(commandEncoder: MTLComputeCommandEncoder, at time: Float) {
            let bufs = [
                mem.physicsFields.physicsFieldBuffer,
                mem.rigidBodies.massBuffer,
                mem.rigidBodies.centerOfMassBuffer,
                mem.rigidBodies.forceBuffer,
                mem.rigidBodies.torqueBuffer,
            ]
            commandEncoder.setBuffers(bufs, offsets: [Int](repeating: 0, count: bufs.count), range: 0..<bufs.count)

            var time = time
            commandEncoder.setBytes(&time, length: MemoryLayout<Float>.size, index: bufs.count)
        }
    }
}
