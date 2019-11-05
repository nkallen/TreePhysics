import Foundation
import MetalKit
import Metal
import SceneKit
import ShaderTypes

final class UpdateCompositeBodies: MetalKernelEncoder {
    private let argumentEncoder: ArgumentEncoder

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, memoryLayoutManager: MemoryLayoutManager) {
        let library = device.makeDefaultLibrary()!
        let constantValues = MTLFunctionConstantValues()
        var rangeCount = memoryLayoutManager.rigidBodies.ranges.count
        constantValues.setConstantValue(&rangeCount, type: .int, index: FunctionConstantIndex.rangeCount.rawValue)
        let function = try! library.makeFunction(name: "updateCompositeBodies", constantValues: constantValues)

        self.argumentEncoder = ArgumentEncoder(memoryLayoutManager: memoryLayoutManager)

        super.init(device: device, function: function)
    }

    func encode(commandBuffer: MTLCommandBuffer) {
        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.setComputePipelineState(computePipelineState)
        commandEncoder.label  = "Update Composite Bodies"
        argumentEncoder.encode(commandEncoder: commandEncoder)

        let maxWidth = argumentEncoder.mem.rigidBodies.ranges.last!.count

        let threadGroupWidth = computePipelineState.maxTotalThreadsPerThreadgroup
        let threadsPerThreadgroup = MTLSizeMake(threadGroupWidth, 1, 1)
        let threadsPerGrid = MTLSize(
            width: maxWidth,
            height: 1,
            depth: 1)

        commandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        commandEncoder.endEncoding()
    }
}

extension UpdateCompositeBodies {
    class ArgumentEncoder {
        let mem: MemoryLayoutManager

        init(memoryLayoutManager: MemoryLayoutManager) {
            self.mem = memoryLayoutManager
        }

        func encode(commandEncoder: MTLComputeCommandEncoder) {
            let bufs = [
                mem.rigidBodies.childCountBuffer,
                mem.rigidBodies.childIndexBuffer,
                mem.rigidBodies.parentIdBuffer,
                mem.rigidBodies.climberCountBuffer,
                mem.rigidBodies.massBuffer,
                mem.rigidBodies.pivotBuffer,
                mem.rigidBodies.forceBuffer,
                mem.rigidBodies.torqueBuffer,
                mem.rigidBodies.centerOfMassBuffer,
                mem.rigidBodies.inertiaTensorBuffer,

                mem.compositeBodies.massBuffer,
                mem.compositeBodies.forceBuffer,
                mem.compositeBodies.torqueBuffer,
                mem.compositeBodies.centerOfMassBuffer,
                mem.compositeBodies.inertiaTensorBuffer,
            ]
            commandEncoder.setBuffers(bufs, offsets: [Int](repeating: 0, count: bufs.count), range: 0..<bufs.count)

            let ranges = mem.rigidBodies.ranges.reversed()
            var start = ranges.first!.upperBound
            commandEncoder.setBytes(&start, length: MemoryLayout<UInt32>.stride * ranges.count, index: bufs.count)
            var deltas = ranges.map { range in
                UInt16(range.upperBound - range.lowerBound)
            }
            commandEncoder.setBytes(&deltas, length: MemoryLayout<UInt16>.stride * deltas.count, index: bufs.count+1)
        }
    }
}
