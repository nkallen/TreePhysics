import Foundation
import MetalKit
import Metal

final class UpdateCompositeBodiesKernel: MetalKernel {
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
        var childIds: TupleType = (0,0,0,0,0)
        memcpy(&childIds, childRigidBodyIndices, childRigidBodyIndices.count)

        let strct = RigidBodyStruct(
            parentId: parentId,
            childIds: childIds,
            childCount: ushort(childRigidBodies.count),
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
