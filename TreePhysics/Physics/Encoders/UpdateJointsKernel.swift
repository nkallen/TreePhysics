import Foundation
import MetalKit
import Metal

final class UpdateJointsKernel: MetalKernel {
    let rigidBodiesBuffer: MTLBuffer
    let compositeBodiesBuffer: MTLBuffer
    let jointsBuffer: MTLBuffer
    let numJoints: Int
    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, rigidBodiesBuffer: MTLBuffer, compositeBodiesBuffer: MTLBuffer, jointsBuffer: MTLBuffer, numJoints: Int) {
        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.jointsBuffer = jointsBuffer
        self.compositeBodiesBuffer = compositeBodiesBuffer
        self.numJoints = numJoints

        super.init(device: device, name: "updateJoints")
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
        let threadsPerGrid = MTLSize(
            width: numJoints,
            height: 1,
            depth: 1)

        commandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        commandEncoder.endEncoding()
    }

    static func buffer(count: Int, device: MTLDevice) -> MTLBuffer {
        // FIXME make storage private
        let buffer = device.makeBuffer(length: count * MemoryLayout<JointStruct>.stride, options: [.storageModeShared])!
        let jointStructs = UnsafeMutableRawPointer(buffer.contents()).bindMemory(to: JointStruct.self, capacity: count)
        for i in 0..<count {
            jointStructs[i] = JointStruct(θ: float3x3(0), k: float(200))
        }
        return buffer
    }
}
