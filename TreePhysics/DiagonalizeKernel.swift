import Foundation
import MetalKit
import Metal

// The 256 byte aligned size of our uniform structure
let alignedUniformsSize = (MemoryLayout<float3x3>.size & ~0xFF) + 0x100
let maxBuffersInFlight = 3

class MetalKernel {
    let device: MTLDevice
    let computePipelineState: MTLComputePipelineState

    init(device: MTLDevice, name: String) {
        self.device = device
        let library = device.makeDefaultLibrary()!
        let kernelFunction = library.makeFunction(name: name)!
        self.computePipelineState = try! device.makeComputePipelineState(function: kernelFunction)
    }
}

final class UpdateCompositeBodiesKernel {
    let updateCompositeBodiesInParallel: UpdateCompositeBodiesInParallelKernel
    let updateCompositeBodiesSequentially: UpdateCompositeBodiesSequentiallyKernel

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, root: RigidBody) {
        let (count, rigidBodiesBuffer, ranges) = UpdateCompositeBodiesKernel.buffer(root: root)
        let compositeBodiesBuffer = device.makeBuffer(
            length: MemoryLayout<CompositeBodyStruct>.stride * count,
            options: [.storageModePrivate])!
//        [(0, 243), (243, 81), (324, 27), (351, 9), (360, 3), (363, 1)]

        var parallelRanges = ranges
        var sequentialTotal = 0
        var sequentialStart = Int.max
        for (offset, count) in ranges.reversed() {
            if (sequentialTotal + count > UpdateCompositeBodiesSequentiallyKernel.maxBodiesPerInvocation) { break }

            sequentialTotal += count
            sequentialStart = offset
            _ = parallelRanges.popLast()
        }
        let sequentialRange = (sequentialStart, sequentialTotal)

        self.updateCompositeBodiesInParallel = UpdateCompositeBodiesInParallelKernel(device: device, rigidBodiesBuffer: rigidBodiesBuffer, ranges: parallelRanges, compositeBodiesBuffer: compositeBodiesBuffer)
        self.updateCompositeBodiesSequentially = UpdateCompositeBodiesSequentiallyKernel(device: device, rigidBodiesBuffer: rigidBodiesBuffer, range: sequentialRange, compositeBodiesBuffer: compositeBodiesBuffer)
    }

    func encode(commandBuffer: MTLCommandBuffer) {
        self.updateCompositeBodiesInParallel.encode(commandBuffer: commandBuffer)
        self.updateCompositeBodiesSequentially.encode(commandBuffer: commandBuffer)
    }

    static func buffer(root: RigidBody, device: MTLDevice = MTLCreateSystemDefaultDevice()!) -> (Int, MTLBuffer, [(Int, Int)]) {
        var allRigidBodies: [RigidBody] = []
        var allStragglers: [RigidBody] = []
        var rangesOfWork: [(Int, Int)] = []
        for (parallel, stragglers) in root.levels {
            let range = (allRigidBodies.count, parallel.count)
            allRigidBodies.append(contentsOf: parallel)
            allStragglers.append(contentsOf: stragglers)
            rangesOfWork.append(range)
        }
        let all = allRigidBodies + allStragglers
        return (all.count, buffer(flattened: all, device: device), rangesOfWork)
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

final class UpdateCompositeBodiesInParallelKernel: MetalKernel {
    let rigidBodiesBuffer: MTLBuffer
    let compositeBodiesBuffer: MTLBuffer
    let ranges: [(Int, Int)]

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, rigidBodiesBuffer: MTLBuffer, ranges: [(Int, Int)], compositeBodiesBuffer: MTLBuffer) {
        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.ranges = ranges
        self.compositeBodiesBuffer = compositeBodiesBuffer
        super.init(device: device, name: "updateCompositeBodies")
    }

    func encode(commandBuffer: MTLCommandBuffer) {
        var i = 0
        for (gridOrigin, threadsPerGrid) in ranges {
            print(gridOrigin, threadsPerGrid)
            let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
            commandEncoder.setComputePipelineState(computePipelineState)
            commandEncoder.label  = "Update Composite Bodies in Parallel, Round \(i)"
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
    }
}

final class UpdateCompositeBodiesSequentiallyKernel: MetalKernel {
    static let maxBodiesPerInvocation = 32

    let rigidBodiesBuffer: MTLBuffer
    let compositeBodiesBuffer: MTLBuffer
    let range: (Int, Int)

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, rigidBodiesBuffer: MTLBuffer, range: (Int, Int), compositeBodiesBuffer: MTLBuffer) {
        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.compositeBodiesBuffer = compositeBodiesBuffer
        self.range = range
        super.init(device: device, name: "updateCompositeBodies2")
    }

    func encode(commandBuffer: MTLCommandBuffer) {
        var (gridOrigin, threadsPerGridWidth) = range

        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.setComputePipelineState(computePipelineState)
        commandEncoder.label  = "Update Composite Bodies Sequentially"
        commandEncoder.setBuffer(rigidBodiesBuffer, offset: 0, index: BufferIndex.rigidBodies.rawValue)
        commandEncoder.setBuffer(compositeBodiesBuffer, offset: 0, index: BufferIndex.compositeBodies.rawValue)
        commandEncoder.setThreadgroupMemoryLength(MemoryLayout<RigidBodyStruct>.stride * UpdateCompositeBodiesSequentiallyKernel.maxBodiesPerInvocation, index: ThreadGroupIndex.rigidBodies.rawValue)
        commandEncoder.setThreadgroupMemoryLength(MemoryLayout<CompositeBodyStruct>.stride * UpdateCompositeBodiesSequentiallyKernel.maxBodiesPerInvocation, index: ThreadGroupIndex.compositeBodies.rawValue)
        commandEncoder.setBytes(&gridOrigin, length: MemoryLayout<Int>.size, index: BufferIndex.gridOrigin.rawValue)

        let width = computePipelineState.maxTotalThreadsPerThreadgroup
        let threadsPerThreadgroup = MTLSizeMake(width, 1, 1)
        let threadsPerGrid = MTLSize(
            width: threadsPerGridWidth,
            height: 1,
            depth: 1)

        commandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        commandEncoder.endEncoding()
    }
}
