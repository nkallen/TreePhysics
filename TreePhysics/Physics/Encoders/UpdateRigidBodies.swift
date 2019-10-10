import Foundation
import MetalKit
import Metal
import ShaderTypes

final class UpdateRigidBodies: MetalKernelEncoder {
    let rigidBodiesBuffer: MTLBuffer
    let compositeBodiesBuffer: MTLBuffer
    let jointsBuffer: MTLBuffer
    let ranges: [Range<Int>]

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, rigidBodiesBuffer: MTLBuffer, compositeBodiesBuffer: MTLBuffer, jointsBuffer: MTLBuffer, ranges: [Range<Int>]) {
        precondition(ranges.count > 0)

        let library = device.makeDefaultLibrary()!
        let constantValues = MTLFunctionConstantValues()
        var rangeCount = ranges.count
        constantValues.setConstantValue(&rangeCount, type: .int, index: FunctionConstantIndex.rangeCount.rawValue)
        let function = try! library.makeFunction(name: "updateRigidBodies", constantValues: constantValues)

        self.ranges = ranges.reversed()

        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.jointsBuffer = jointsBuffer
        self.compositeBodiesBuffer = compositeBodiesBuffer
        super.init(device: device, function: function)
    }

    func encode(commandBuffer: MTLCommandBuffer) {
        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.setComputePipelineState(computePipelineState)
        commandEncoder.label  = "Update Rigid Bodies"
        commandEncoder.setBuffer(rigidBodiesBuffer, offset: 0, index: BufferIndex.rigidBodies.rawValue)
        commandEncoder.setBuffer(compositeBodiesBuffer, offset: 0, index: BufferIndex.compositeBodies.rawValue)
        commandEncoder.setBuffer(jointsBuffer, offset: 0, index: BufferIndex.joints.rawValue)
        var ranges: [SIMD2<Int32>] = self.ranges.map { SIMD2<Int32>(Int32($0.lowerBound), Int32($0.upperBound)) }
        commandEncoder.setBytes(&ranges, length: MemoryLayout<SIMD2<Int32>>.stride * ranges.count, index: BufferIndex.ranges.rawValue)

        let maxWidth = self.ranges.last!.count

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
