import Foundation
import MetalKit
import Metal
import SceneKit
import ShaderTypes

public final class UpdateFreeBodies: MetalKernelEncoder {
    private let argumentEncoder: ArgumentEncoder

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, memoryLayoutManager: MemoryLayoutManager) {
        let library = device.makeDefaultLibrary()!
        let function = library.makeFunction(name: "encodeUpdateFreeBodies")!

        self.argumentEncoder = ArgumentEncoder(memoryLayoutManager: memoryLayoutManager, function: function)

        super.init(device: device, function: function)
    }

    func encode(commandBuffer: MTLCommandBuffer, at time: Float) {
        let encodeUpdateFreeBodiesCommandEncoder = commandBuffer.makeComputeCommandEncoder()!
        encodeUpdateFreeBodiesCommandEncoder.setComputePipelineState(computePipelineState)
        encodeUpdateFreeBodiesCommandEncoder.label  = "Encode Update Free Bodies"
        argumentEncoder.encode(commandEncoder: encodeUpdateFreeBodiesCommandEncoder, at: time)

        let threadGroupWidth = computePipelineState.maxTotalThreadsPerThreadgroup
        let threadsPerThreadgroup = MTLSizeMake(threadGroupWidth, 1, 1)
        let threadsPerGrid = MTLSize(
            width: 1,
            height: 1,
            depth: 1)

        encodeUpdateFreeBodiesCommandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        encodeUpdateFreeBodiesCommandEncoder.endEncoding()

        let updateFreeBodiesCommandEncoder = commandBuffer.makeComputeCommandEncoder()!
        updateFreeBodiesCommandEncoder.label = "Update Free Bodies"
        updateFreeBodiesCommandEncoder.executeCommands(in: argumentEncoder.icb, with: NSRange(location: 0, length: 1))
        updateFreeBodiesCommandEncoder.endEncoding()
    }
}

extension UpdateFreeBodies {
    class ArgumentEncoder {
        let mem: MemoryLayoutManager
        let function: MTLFunction
        let computePipelineState: MTLComputePipelineState
        let icb: MTLIndirectCommandBuffer

        init(memoryLayoutManager: MemoryLayoutManager, function: MTLFunction) {
            self.mem = memoryLayoutManager
            self.function = function
            let library = function.device.makeDefaultLibrary()!
            let f = library.makeFunction(name: "updateFreeBodies")!
            let descriptor = MTLComputePipelineDescriptor()
            descriptor.supportIndirectCommandBuffers = true
            descriptor.computeFunction = f
            self.computePipelineState = try! f.device.makeComputePipelineState(descriptor: descriptor, options: [], reflection: nil)

            let icbDescriptor = MTLIndirectCommandBufferDescriptor()
            icbDescriptor.commandTypes = .concurrentDispatchThreads
            icbDescriptor.inheritBuffers = true
            let icb = f.device.makeIndirectCommandBuffer(descriptor: icbDescriptor, maxCommandCount: 1, options: .storageModeShared)!
            let command = icb.indirectComputeCommand(at: 0)
            command.setComputePipelineState(computePipelineState)
            self.icb = icb
        }

        func encode(commandEncoder: MTLComputeCommandEncoder, at time: Float) {
            let argumentEncoder = function.makeArgumentEncoder(bufferIndex: 0)
            let buffer = commandEncoder.device.makeBuffer(length: argumentEncoder.encodedLength, options: .storageModeShared)!
            argumentEncoder.setArgumentBuffer(buffer, offset: 0)
            argumentEncoder.setIndirectCommandBuffer(icb, index: 0)

            let bufs = [
                mem.freeBodies.indexBuffer,
                mem.freeBodies.countBuffer,
                mem.freeBodies.massBuffer,
                mem.freeBodies.forceBuffer,
                mem.freeBodies.torqueBuffer,
                mem.freeBodies.inertiaTensorBuffer,

                mem.rigidBodies.localInertiaTensorBuffer,
                mem.rigidBodies.centerOfMassBuffer,
                mem.rigidBodies.orientationBuffer,
                mem.rigidBodies.angularMomentumBuffer,
                mem.rigidBodies.velocityBuffer,
                mem.rigidBodies.accelerationBuffer,
                mem.rigidBodies.angularVelocityBuffer,
                mem.rigidBodies.inertiaTensorBuffer,
            ]
            argumentEncoder.setBuffers(bufs, offsets: [Int](repeating: 0, count: bufs.count), range: 1..<bufs.count+1)
            commandEncoder.setBuffer(buffer, offset: 0, index: 0)
//            var time = time
//            commandEncoder.setBytes(&time, length: MemoryLayout<Float>.size, index: 1)

            commandEncoder.useResource(icb, usage: .read)
            commandEncoder.useResources(bufs, usage: .write)
        }
    }
}
