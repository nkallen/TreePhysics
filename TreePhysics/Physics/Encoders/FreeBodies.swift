import Foundation
import MetalKit
import Metal
import SceneKit
import ShaderTypes

public final class FreeBodies: MetalKernelEncoder {
    private let argumentEncoder: ArgumentEncoder

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, memoryLayoutManager: MemoryLayoutManager) {
        let library = device.makeDefaultLibrary()!
        let function = library.makeFunction(name: "encodeFreeBodies")!

        self.argumentEncoder = ArgumentEncoder(memoryLayoutManager: memoryLayoutManager, function: function)

        super.init(device: device, function: function)
    }

    func encode(commandBuffer: MTLCommandBuffer) {
        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.setComputePipelineState(computePipelineState)
        commandEncoder.label  = "Encode Free Bodies"
        argumentEncoder.encode(commandEncoder: commandEncoder)

        let threadGroupWidth = computePipelineState.maxTotalThreadsPerThreadgroup
        let threadsPerThreadgroup = MTLSizeMake(threadGroupWidth, 1, 1)
        let threadsPerGrid = MTLSize(
            width: 1,
            height: 1,
            depth: 1)

        commandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        commandEncoder.endEncoding()

        let commandEncoder2 = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder2.label = "Free Bodies"
        commandEncoder2.executeCommands(in: argumentEncoder.icb, with: NSRange(location: 0, length: 1))
        commandEncoder2.endEncoding()
    }
}

extension FreeBodies {
    class ArgumentEncoder {
        let mem: MemoryLayoutManager
        let function: MTLFunction
        let computePipelineState: MTLComputePipelineState
        let icb: MTLIndirectCommandBuffer

        init(memoryLayoutManager: MemoryLayoutManager, function: MTLFunction) {
            self.mem = memoryLayoutManager
            self.function = function
            let library = function.device.makeDefaultLibrary()!
            let f = library.makeFunction(name: "freeBodies")!
            let descriptor = MTLComputePipelineDescriptor()
            descriptor.supportIndirectCommandBuffers = true
            descriptor.computeFunction = f
            self.computePipelineState = try! f.device.makeComputePipelineState(descriptor: descriptor, options: [], reflection: nil)

            let descriptor2 = MTLIndirectCommandBufferDescriptor()
            descriptor2.commandTypes = .concurrentDispatchThreads
            descriptor2.inheritBuffers = true
            let icb = f.device.makeIndirectCommandBuffer(descriptor: descriptor2, maxCommandCount: 1, options: .storageModeShared)!
            let command = icb.indirectComputeCommand(at: 0)
            command.setComputePipelineState(computePipelineState)
            self.icb = icb
        }

        func encode(commandEncoder: MTLComputeCommandEncoder) {
            let argumentEncoder = function.makeArgumentEncoder(bufferIndex: 0)
            let buffer = commandEncoder.device.makeBuffer(length: argumentEncoder.encodedLength, options: .storageModeShared)!
            argumentEncoder.setArgumentBuffer(buffer, offset: 0)
            argumentEncoder.setIndirectCommandBuffer(icb, index: 0)

            let bufs = [
                mem.freeBodies.indexBuffer,
                mem.rigidBodies.firstChildIdBuffer,
                mem.rigidBodies.parentIdBuffer,
                mem.rigidBodies.childIndexBuffer,
                mem.rigidBodies.childCountBuffer,
            ]
            argumentEncoder.setBuffers(bufs, offsets: [Int](repeating: 0, count: bufs.count), range: 1..<bufs.count+1)
            commandEncoder.setBuffer(buffer, offset: 0, index: 0)
            commandEncoder.setBuffer(mem.freeBodies.countBuffer, offset: 0, index: 1)
            commandEncoder.useResource(icb, usage: .read)
            commandEncoder.useResources(bufs, usage: .write)
        }
    }
}
