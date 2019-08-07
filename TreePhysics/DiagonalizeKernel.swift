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

    init(device: MTLDevice, name: String) {
        self.device = device
        let library = device.makeDefaultLibrary()!
        let kernelFunction = library.makeFunction(name: name)!
        self.computePipelineState = try! device.makeComputePipelineState(function: kernelFunction)
        self.commandQueue = device.makeCommandQueue()!
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
    let ranges: [(Int, Int)]

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, root: RigidBody) {
        let (rigidBodiesBuffer, ranges) = UpdateCompositeBodiesKernel.buffer(root: root, device: device)
        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.ranges = ranges
        self.compositeBodiesBuffer = device.makeBuffer(
            length: MemoryLayout<CompositeBodyStruct>.stride * num,
            options: [isRunningTests ? .storageModeShared : .storageModePrivate])!
        super.init(device: device, name: "updateCompositeBodies")
    }

    func run(cb: @escaping (MTLBuffer) -> ()) {
        let commandBuffer = commandQueue.makeCommandBuffer()!
        commandBuffer.addCompletedHandler{ [weak self] commandBuffer in
            if let strongSelf = self {
                cb(strongSelf.compositeBodiesBuffer)
            }
        }

        var i = 0
        for (gridOrigin, threadsPerGrid) in ranges {
            print(gridOrigin, threadsPerGrid)
            let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
            commandEncoder.setComputePipelineState(computePipelineState)
            commandEncoder.label  = "Update Composite Bodies \(i)"
            commandEncoder.setBuffer(rigidBodiesBuffer, offset: 0, index: BufferIndex.rigidBodies.rawValue)
            commandEncoder.setBuffer(compositeBodiesBuffer, offset: 0, index: BufferIndex.compositeBodies.rawValue)
            var gridOrigin = gridOrigin
            commandEncoder.setBytes(&gridOrigin, length: MemoryLayout<Int>.size, index: BufferIndex.gridOrigin.rawValue)

            let width = computePipelineState.maxTotalThreadsPerThreadgroup
            let threadsPerThreadgroup = MTLSizeMake(width, 1, 1)
            let threadsPerGrid = MTLSize(
                width: threadsPerGrid,
                height: 1,
                depth: 1)

            commandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
            commandEncoder.endEncoding()

            i += 1
        }

        commandBuffer.commit()
    }

    static func buffer(root: RigidBody, device: MTLDevice = MTLCreateSystemDefaultDevice()!) -> (MTLBuffer, [(Int, Int)]) {
        var allRigidBodies: [RigidBody] = []
        var allStragglers: [RigidBody] = []
        var rangesOfWork: [(Int, Int)] = []
        for (parallel, stragglers) in root.levels {
            let range = (allRigidBodies.count, parallel.count)
            allRigidBodies.append(contentsOf: parallel)
            allStragglers.append(contentsOf: stragglers)
            rangesOfWork.append(range)
        }
        return (buffer(flattened: allRigidBodies + allStragglers, device: device), rangesOfWork)
    }

    static func buffer(flattened: [RigidBody], device: MTLDevice) -> MTLBuffer {
        let buffer = device.makeBuffer(length: flattened.count * MemoryLayout<RigidBodyStruct>.stride, options: [.storageModeShared])!
        var index: [RigidBody:Int32] = [:]
        for (i, rigidBody) in flattened.enumerated() {
            index[rigidBody] = Int32(i)
        }
        let rigidBodyStructs = UnsafeMutableRawPointer(buffer.contents())!.bindMemory(to: RigidBodyStruct.self, capacity: flattened.count)
        for (i, rigidBody) in flattened.enumerated() {
            rigidBodyStructs[i] = `struct`(rigidBody: rigidBody, index: index)
        }
        print(index)
        return buffer
    }

    typealias TupleType = (Int32, Int32, Int32, Int32, Int32)

    private static func `struct`(rigidBody: RigidBody, index: [RigidBody:Int32]) -> RigidBodyStruct {
        let parentId: Int32
        if let parentJoint = rigidBody.parentJoint {
            parentId = index[parentJoint.parentRigidBody]!
        } else {
            parentId = -1
        }
        let childRigidBodies = rigidBody.childJoints.map { $0.childRigidBody }
        let childRigidBodyIndices = childRigidBodies.map { index[$0] }
        assert(childRigidBodies.count < 5)
        let childIds = UnsafeMutablePointer<TupleType>.allocate(capacity: 1)
        memcpy(childIds, childRigidBodyIndices, childRigidBodyIndices.count)

        let strct = RigidBodyStruct(
            parentId: parentId,
            childIds: childIds.pointee,
            childCount: uint(childRigidBodies.count),
            position: rigidBody.position,
            mass: rigidBody.mass,
            length: rigidBody.length,
            radius: rigidBody.radius,
            inertiaTensor: rigidBody.inertiaTensor,
            force: rigidBody.force,
            torque: rigidBody.torque,
            centerOfMass: rigidBody.centerOfMass)
        return strct
    }
}

final class UpdateCompositeBodies2Kernel: MetalKernel {
    let rigidBodiesBuffer: MTLBuffer
    let compositeBodiesBuffer: MTLBuffer
    let range: (Int, Int)

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, rigidBodiesBuffer: MTLBuffer, range: (Int, Int), compositeBodiesBuffer: MTLBuffer) {
        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.compositeBodiesBuffer = compositeBodiesBuffer
        self.range = range
        super.init(device: device, name: "updateCompositeBodies2")
    }

    func run(cb: @escaping () -> ()) {
        let commandBuffer = commandQueue.makeCommandBuffer()!
        commandBuffer.addCompletedHandler { commandBuffer in
            cb()
        }
        var (gridOrigin, threadsPerGridWidth) = range

        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.setComputePipelineState(computePipelineState)
        commandEncoder.label  = "Update Composite Bodies Serial"
        commandEncoder.setBuffer(rigidBodiesBuffer, offset: 0, index: BufferIndex.rigidBodies.rawValue)
        commandEncoder.setBuffer(compositeBodiesBuffer, offset: 0, index: BufferIndex.compositeBodies.rawValue)
        commandEncoder.setThreadgroupMemoryLength(MemoryLayout<RigidBodyStruct>.stride * num, index: ThreadGroupIndex.rigidBodies.rawValue)
        commandEncoder.setThreadgroupMemoryLength(MemoryLayout<CompositeBodyStruct>.stride * num, index: ThreadGroupIndex.compositeBodies.rawValue)
        commandEncoder.setBytes(&gridOrigin, length: MemoryLayout<Int>.size, index: BufferIndex.gridOrigin.rawValue)

        let width = computePipelineState.maxTotalThreadsPerThreadgroup
        let threadsPerThreadgroup = MTLSizeMake(width, 1, 1)
        let threadsPerGrid = MTLSize(
            width: threadsPerGridWidth,
            height: 1,
            depth: 1)

        commandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        commandEncoder.endEncoding()

        commandBuffer.commit()
    }
}
