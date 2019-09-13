import Foundation
import MetalKit
import Metal
import SceneKit
import ShaderTypes

final class UpdateCompositeBodies: MetalKernelEncoder {
    let rigidBodiesBuffer: MTLBuffer
    let compositeBodiesBuffer: MTLBuffer
    let ranges: [Range<Int>]

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, rigidBodiesBuffer: MTLBuffer, ranges: [Range<Int>], compositeBodiesBuffer: MTLBuffer) {
        precondition(ranges.count > 0)

        self.ranges = ranges
        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.compositeBodiesBuffer = compositeBodiesBuffer

        let library = device.makeDefaultLibrary()!
        let constantValues = MTLFunctionConstantValues()
        var rangeCount = ranges.count
        constantValues.setConstantValue(&rangeCount, type: .int, index: FunctionConstantIndex.rangeCount.rawValue)
        let function = try! library.makeFunction(name: "updateCompositeBodies", constantValues: constantValues)

        super.init(device: device, function: function)
    }

    func encode(commandBuffer: MTLCommandBuffer) {
        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.setComputePipelineState(computePipelineState)
        commandEncoder.label  = "Update Composite Bodies"
        commandEncoder.setBuffer(rigidBodiesBuffer, offset: 0, index: BufferIndex.rigidBodies.rawValue)
        commandEncoder.setBuffer(compositeBodiesBuffer, offset: 0, index: BufferIndex.compositeBodies.rawValue)
        var ranges = self.ranges.map { int2(Int32($0.lowerBound), Int32($0.upperBound)) }
        commandEncoder.setBytes(&ranges, length: MemoryLayout<int2>.stride * ranges.count, index: BufferIndex.ranges.rawValue)

        let maxWidth = self.ranges.first!.count

        let threadGroupWidth = computePipelineState.maxTotalThreadsPerThreadgroup
        let threadsPerThreadgroup = MTLSizeMake(threadGroupWidth, 1, 1)
        let threadsPerGrid = MTLSize(
            width: maxWidth,
            height: 1,
            depth: 1)

        commandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        commandEncoder.endEncoding()
    }

    static func rigidBodiesBuffer(root: Internode, device: MTLDevice) -> ([RigidBody], MTLBuffer, [Range<Int>]) {
        var rangesOfWork: [Range<Int>] = []
        let levels = root.levels()
        var offset = 0
        var id = 0
        let index = RigidBodyDict<Int>()
        var allClimbers: [RigidBody] = []
        var rigidBodies: [RigidBody] = []

        // Step 1: Determine the buffer memory layout (i.e., the index and the ranges of work)
        for level in levels {
            for unitOfWork in level {
                allClimbers.append(contentsOf: unitOfWork.climbers)
                index[unitOfWork.rigidBody] = id
                id += 1
            }

            let range = offset..<(offset+level.count)
            rangesOfWork.append(range)
            offset += level.count
        }
        for rigidBody in allClimbers {
            index[rigidBody] = id
            id += 1
        }
        index[root] = id

        // Step 1: Allocate the buffer
        let count = offset + allClimbers.count + 1 // +1 for root
        let buffer = device.makeBuffer(length: count * MemoryLayout<RigidBodyStruct>.stride, options: [.storageModeShared])!

        // Step 3: Store data into the buffer
        let rigidBodyStructs = UnsafeMutableRawPointer(buffer.contents())!.bindMemory(to: RigidBodyStruct.self, capacity: count)
        for level in levels {
            for unitOfWork in level {
                let id = index[unitOfWork.rigidBody]!
                rigidBodyStructs[id] = `struct`(rigidBody: unitOfWork.rigidBody, climbers: unitOfWork.climbers, index: index)
                rigidBodies.append(unitOfWork.rigidBody)
            }
        }
        for rigidBody in allClimbers {
            let id = index[rigidBody]!
            rigidBodyStructs[id] = `struct`(rigidBody: rigidBody, index: index)
            rigidBodies.append(rigidBody)
        }
        rigidBodyStructs[index[root]!] = `struct`(rigidBody: root, index: index)
        rigidBodies.append(root)
        return (rigidBodies, buffer, rangesOfWork)
    }

    static func compositeBodiesBuffer(count: Int, device: MTLDevice) -> MTLBuffer {
        let buffer = device.makeBuffer(length: count * MemoryLayout<CompositeBodyStruct>.stride, options: [.storageModePrivate])!
        return buffer
    }

    typealias ChildIdsType = (Int32, Int32, Int32)

    private static func `struct`(rigidBody: RigidBody, climbers: [RigidBody] = [], index: RigidBodyDict<Int>) -> RigidBodyStruct {
        // Parent
        let parentId: Int32
        if let parentJoint = rigidBody.parentJoint {
            parentId = Int32(index[parentJoint.parentRigidBody]!)
        } else {
            parentId = -1
        }

        // Child
        let childRigidBodies = rigidBody.childJoints.map { $0.childRigidBody }
        assert(childRigidBodies.count <= 5)
        var childRigidBodyIndices: [Int32] = childRigidBodies.map { Int32(index[$0]!) }
        var childIds: ChildIdsType = (0,0,0)
        memcpy(&childIds, &childRigidBodyIndices, childRigidBodyIndices.count * MemoryLayout<Int32>.stride)

        // Climbers
//        assert(climbers.count <= 10) FIXME
        let climberOffset = Int32(climbers.first.map { index[$0]! } ?? 0)

        let strct = RigidBodyStruct(
            parentId: parentId,
            childIds: childIds,
            climberOffset: climberOffset,
            childCount: ushort(childRigidBodies.count),
            climberCount: ushort(climbers.count),
            mass: rigidBody.mass,
            // FIXME cast
            length: (rigidBody as! Internode).length,
            radius: (rigidBody as! Internode).radius,

            jointStiffness: 200.0,
            jointLocalRotation: rigidBody.parentJoint.map { float3x3($0.rotation_local) } ?? matrix_identity_float3x3,

            // FIXME rename
            position: rigidBody.translation,
            rotation: float3x3(rigidBody.rotation),
            inertiaTensor: rigidBody.inertiaTensor,
            centerOfMass: rigidBody.centerOfMass,

            force: rigidBody.force,
            torque: rigidBody.torque)
        return strct
    }
}
