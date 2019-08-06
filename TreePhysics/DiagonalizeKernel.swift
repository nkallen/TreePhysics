import Foundation
import MetalKit
import Metal

// The 256 byte aligned size of our uniform structure
let alignedUniformsSize = (MemoryLayout<float3x3>.size & ~0xFF) + 0x100
let maxBuffersInFlight = 3
let num = 4096

class MetalKernel {
    let device: MTLDevice
    let computePipelineState: MTLComputePipelineState
    let commandQueue: MTLCommandQueue
    let captureManager: MTLCaptureManager

    init(device: MTLDevice, name: String) {
        self.device = device
        let library = device.makeDefaultLibrary()!
        let kernelFunction = library.makeFunction(name: name)!
        self.computePipelineState = try! device.makeComputePipelineState(function: kernelFunction)
        self.commandQueue = device.makeCommandQueue()!

        self.captureManager = MTLCaptureManager.shared()
        captureManager.startCapture(commandQueue: commandQueue)
        print(captureManager.isCapturing)
    }
}

class DiagonalizeKernel: MetalKernel {
    let buffer: MTLBuffer
    let outBuffer: MTLBuffer
    var bufferOffset: Int

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!) {
        self.buffer = device.makeBuffer(length: MemoryLayout<float3x3>.size, options: [.storageModeShared])!
        self.outBuffer = device.makeBuffer(length: MemoryLayout<float3>.size * num, options: [.storageModeShared])!
        self.bufferOffset = 0
        super.init(device: device, name: "diagonalize")
    }

    func run(_ matrix: float3x3, cb: @escaping (MTLBuffer) -> ()) {
        let foo = UnsafeMutableRawPointer(buffer.contents() + bufferOffset).bindMemory(to: float3x3.self, capacity: 1)
        foo[0] = matrix

        let commandBuffer = commandQueue.makeCommandBuffer()!
        commandBuffer.addCompletedHandler{ [weak self] commandBuffer in
            if let strongSelf = self {
                cb(strongSelf.outBuffer)
            }
        }

        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.setComputePipelineState(computePipelineState)
        commandEncoder.label  = "Diagonalize"
        commandEncoder.setBuffer(buffer, offset: bufferOffset, index: 0)
        commandEncoder.setBuffer(outBuffer, offset: 0, index: 1)

        let width = computePipelineState.threadExecutionWidth
        let height = computePipelineState.maxTotalThreadsPerThreadgroup / width
        let threadsPerThreadgroup = MTLSizeMake(width, height, 1)
        let threadsPerGrid = MTLSize(
            width: 1,
            height: num,
            depth: 1)
        commandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        commandEncoder.endEncoding()

        commandBuffer.commit()
    }
}

final class UpdateCompositeBodiesKernel: MetalKernel {
    let rigidBodiesBuffer: MTLBuffer
    let compositeBodiesBuffer: MTLBuffer
    let num: Int

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, rigidBody: RigidBody) {
        let flattened = Array(rigidBody.flattened.reversed())
        self.num = flattened.count
        self.rigidBodiesBuffer = UpdateCompositeBodiesKernel.buffer(flattened: flattened, device: device)
        self.compositeBodiesBuffer = device.makeBuffer(length: MemoryLayout<CompositeBodyStruct>.stride * num, options: [.storageModeShared])!
        super.init(device: device, name: "updateCompositeBodies")
    }

    func run(cb: @escaping (MTLBuffer) -> ()) {
//        let rigidBodiesCache = device.makeBuffer(length: MemoryLayout<RigidBodyStruct>.stride * num, options: [.storageModeShared])!
//        let compositeBodiesCache = device.makeBuffer(length: MemoryLayout<CompositeBodyStruct>.stride * num, options: [.storageModeShared])!
//        let compositeBodiesDone = device.makeBuffer(length: MemoryLayout<Bool>.stride * num, options: [.storageModeShared])!

        let commandBuffer = commandQueue.makeCommandBuffer()!
        commandBuffer.addCompletedHandler{ [weak self] commandBuffer in
            if let strongSelf = self {
                cb(strongSelf.compositeBodiesBuffer)
            }
        }

        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.setComputePipelineState(computePipelineState)
        commandEncoder.label  = "Update Composite Bodies"
        commandEncoder.setBuffer(rigidBodiesBuffer, offset: 0, index: BufferIndex.rigidBodies.rawValue)
        commandEncoder.setBuffer(compositeBodiesBuffer, offset: 0, index: BufferIndex.compositeBodies.rawValue)

        let width = computePipelineState.maxTotalThreadsPerThreadgroup
        let threadsPerThreadgroup = MTLSizeMake(width, 1, 1)
        let threadsPerGrid = MTLSize(
            width: num,
            height: 1,
            depth: 1)

//        commandEncoder.setBuffer(rigidBodiesCache, offset: 0, index: 2)
//        commandEncoder.setBuffer(compositeBodiesCache, offset: 0, index: 3)
//        commandEncoder.setBuffer(compositeBodiesDone, offset: 0, index: 4)

        commandEncoder.setThreadgroupMemoryLength(width * MemoryLayout<RigidBody>.stride, index: ThreadGroupIndex.rigidBodies.rawValue)
        commandEncoder.setThreadgroupMemoryLength(width * MemoryLayout<CompositeBody>.stride, index: ThreadGroupIndex.compositeBodies.rawValue)
        commandEncoder.setThreadgroupMemoryLength(width * MemoryLayout<Bool>.stride, index: ThreadGroupIndex.compositeBodiesDone.rawValue)

        commandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        commandEncoder.endEncoding()

        commandBuffer.commit()
        captureManager.stopCapture()
    }

    private static func buffer(flattened: [RigidBody], device: MTLDevice) -> MTLBuffer {
        let device = MTLCreateSystemDefaultDevice()!
        let buffer = device.makeBuffer(length: flattened.count * MemoryLayout<RigidBodyStruct>.stride, options: [.storageModeShared])!
        var index: [RigidBody:Int32] = [:]
        for (i, rigidBody) in flattened.enumerated() {
            index[rigidBody] = Int32(i)
        }
        let rigidBodyStructs = UnsafeMutableRawPointer(buffer.contents())!.bindMemory(to: RigidBodyStruct.self, capacity: flattened.count)
        for (i, rigidBody) in flattened.enumerated() {
            rigidBodyStructs[i] = `struct`(rigidBody: rigidBody, index: index)
            print((i, `struct`(rigidBody: rigidBody, index: index)))
        }
        return buffer
    }

    typealias TupleType = (Int32, Int32, Int32, Int32, Int32)

    private static func `struct`(rigidBody: RigidBody, index: [RigidBody:Int32]) -> RigidBodyStruct {
        let parentGid: Int32
        if let parentJoint = rigidBody.parentJoint {
            parentGid = index[parentJoint.parentRigidBody]!
        } else {
            parentGid = -1
        }
        let childRigidBodies = rigidBody.childJoints.map { $0.childRigidBody }
        let childRigidBodyIndices = childRigidBodies.map { index[$0] }
        assert(childRigidBodies.count < 5)
        let childGids = UnsafeMutablePointer<TupleType>.allocate(capacity: 1)
        memcpy(childGids, childRigidBodyIndices, childRigidBodyIndices.count)

        let strct = RigidBodyStruct(
            parentGid: parentGid,
            childGids: childGids.pointee,
            childCount: uint(childRigidBodies.count),
            mass: rigidBody.mass,
            length: rigidBody.length,
            radius: rigidBody.radius,
            inertiaTensor: rigidBody.inertiaTensor,
            position: rigidBody.position,
            force: rigidBody.force,
            torque: rigidBody.torque,
            centerOfMass: rigidBody.centerOfMass)
        return strct
    }
}
