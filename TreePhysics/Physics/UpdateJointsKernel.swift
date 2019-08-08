import Foundation
import MetalKit
import Metal

final class UpdateJointsKernel: MetalKernel {
    let rigidBodiesBuffer: MTLBuffer
    let compositeBodiesBuffer: MTLBuffer
    let jointsBuffer: MTLBuffer
    let numJoints: Int

    let debugRigidBodies: MTLBuffer
    let debugCompositeBodies: MTLBuffer
    let debugFloats: MTLBuffer
    let debugFloat3s: MTLBuffer
    let debugFloat3x3s: MTLBuffer

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, rigidBodiesBuffer: MTLBuffer, compositeBodiesBuffer: MTLBuffer, jointsBuffer: MTLBuffer, numJoints: Int) {
        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.jointsBuffer = jointsBuffer
        self.compositeBodiesBuffer = compositeBodiesBuffer
        self.numJoints = numJoints
        self.debugRigidBodies = device.makeBuffer(length: 1024, options: [.storageModeShared])!
        self.debugCompositeBodies = device.makeBuffer(length: 1024, options: [.storageModeShared])!
        self.debugFloats = device.makeBuffer(length: 1024, options: [.storageModeShared])!
        self.debugFloat3s = device.makeBuffer(length: 1024, options: [.storageModeShared])!
        self.debugFloat3x3s = device.makeBuffer(length: 1024, options: [.storageModeShared])!
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

        commandEncoder.setBuffer(debugRigidBodies, offset: 0, index: BufferIndex.debugRigidBody.rawValue)
        commandEncoder.setBuffer(debugCompositeBodies, offset: 0, index: BufferIndex.debugCompositeBody.rawValue)
        commandEncoder.setBuffer(debugFloats, offset: 0, index: BufferIndex.debugFloat.rawValue)
        commandEncoder.setBuffer(debugFloat3s, offset: 0, index: BufferIndex.debugFloat3.rawValue)
        commandEncoder.setBuffer(debugFloat3x3s, offset: 0, index: BufferIndex.debugFloat3x3.rawValue)

        /*
        commandBuffer.addCompletedHandler { _ in
            let rigidBodies = UnsafeMutableRawPointer(self.debugRigidBodies.contents()).bindMemory(to: RigidBodyStruct.self, capacity: 1)
            let compositeBodies = UnsafeMutableRawPointer(self.debugCompositeBodies.contents()).bindMemory(to: CompositeBodyStruct.self, capacity: 1)
            let floats = UnsafeMutableRawPointer(self.debugFloats.contents()).bindMemory(to: Float.self, capacity: 1)
            let float3s = UnsafeMutableRawPointer(self.debugFloat3s.contents()).bindMemory(to: float3.self, capacity: 1)
            let float3x3s = UnsafeMutableRawPointer(self.debugFloat3x3s.contents()).bindMemory(to: float3x3.self, capacity: 1)


            print("Rigid bodies:", "\n", rigidBodies[0], "\n", rigidBodies[1])
            print("Composite bodies:", compositeBodies[0])
            print("Float3x3s:", "\n", float3x3s[0])
            print("Float:", "\n", floats[0], floats[1])
//            print("Debug output:", foo[1])
//            fatalError()
        }
 */

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
        let buffer = device.makeBuffer(length: count * MemoryLayout<JointStruct>.stride, options: [.storageModeShared])!
        let jointStructs = UnsafeMutableRawPointer(buffer.contents()).bindMemory(to: JointStruct.self, capacity: count)
        for i in 0..<count {
            jointStructs[i] = JointStruct(θ: float3x3(0), k: 200)
        }
        return buffer
    }
}
