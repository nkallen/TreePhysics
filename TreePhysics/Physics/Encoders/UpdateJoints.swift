import Foundation
import MetalKit
import Metal
import ShaderTypes

final class UpdateJoints: MetalKernelEncoder {
    private let argumentEncoder: ArgumentEncoder

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, memoryLayoutManager: MemoryLayoutManager) {
        self.argumentEncoder = ArgumentEncoder(memoryLayoutManager: memoryLayoutManager)

        super.init(device: device, name: "updateJoints")
    }

    func encode(commandBuffer: MTLCommandBuffer, at time: Float) {
        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.setComputePipelineState(computePipelineState)
        commandEncoder.label  = "Update Joints"
        argumentEncoder.encode(commandEncoder: commandEncoder, at: time)

        let threadGroupWidth = computePipelineState.maxTotalThreadsPerThreadgroup
        let threadsPerThreadgroup = MTLSizeMake(threadGroupWidth, 1, 1)
        let numJointsExcludingRoot = argumentEncoder.mem.rigidBodies.count
        let threadsPerGrid = MTLSize(
            width: numJointsExcludingRoot,
            height: 1,
            depth: 1)

        commandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        commandEncoder.endEncoding()
    }
}

extension UpdateJoints {
    class ArgumentEncoder {
        let mem: MemoryLayoutManager

        init(memoryLayoutManager: MemoryLayoutManager) {
            self.mem = memoryLayoutManager
        }

        func encode(commandEncoder: MTLComputeCommandEncoder, at time: Float) {
            let bufs = [
                mem.joints.thetaBuffer,
                mem.rigidBodies.jointStiffnessBuffer,
                mem.rigidBodies.jointDampingBuffer,

                mem.joints.torqueBuffer,
                mem.joints.inertiaTensorBuffer,
            ]
            commandEncoder.setBuffers(bufs, offsets: [Int](repeating: 0, count: bufs.count), range: 0..<bufs.count)

            var time = time
            commandEncoder.setBytes(&time, length: MemoryLayout<Float>.size, index: bufs.count)
        }
    }
}
