import Foundation
import simd
import MetalKit

class MemoryLayoutManager {
    class RigidBodies {
        let ranges: [Range<Int>]
        let maxClimberCount: Int
        let count: Int
        let childCountBuffer, childIndexBuffer, parentIdBuffer, climberCountBuffer, massBuffer, pivotBuffer, forceBuffer, torqueBuffer, centerOfMassBuffer, inertiaTensorBuffer, jointDampingBuffer, jointStiffnessBuffer, jointRotationBuffer: MTLBuffer

        init(device: MTLDevice, root: ArticulatedRigidBody) {
            var ranges: [Range<Int>] = []
            let levels = root.levels()
            var offset = 0
            var id = 0
            var index = [RigidBody:Int]()

            // Step 1: Determine the buffer memory layout (i.e., the index and the ranges of work)
            var maxClimberCount = 0
            for level in levels {
                for unitOfWork in level {
                    index[unitOfWork.rigidBody] = id
                    id += 1
                    maxClimberCount = max(maxClimberCount, unitOfWork.climbers.count)
                }

                let range = offset..<(offset+level.count)
                ranges.append(range)
                offset += level.count
            }
            self.ranges = ranges
            self.maxClimberCount = maxClimberCount

            for level in levels {
                for (id, unitOfWork) in level.enumerated() {
                    for (i, climber) in unitOfWork.climbers.enumerated() {
                        index[climber] = offset + level.count * i + id
                    }
                }
                offset += level.count * maxClimberCount
            }

            // Step 2: Allocate the shared buffers
            self.count = offset

            let childCountBuffer = device.makeBuffer(length: count * MemoryLayout<UInt8>.stride, options: [.storageModeShared])!
            let childIndexBuffer = device.makeBuffer(length: count * MemoryLayout<UInt8>.stride, options: [.storageModeShared])!
            let parentIdBuffer = device.makeBuffer(length: count * MemoryLayout<UInt16>.stride, options: [.storageModeShared])!
            let climberCountBuffer = device.makeBuffer(length: count * MemoryLayout<UInt8>.stride, options: [.storageModeShared])!
            let massBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModeShared])!
            let pivotBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            let forceBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            let torqueBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            let centerOfMassBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            let inertiaTensorBuffer = device.makeBuffer(length: count * MemoryLayout<matrix_float3x3>.stride, options: [.storageModeShared])!
            let jointDampingBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModeShared])!
            let jointStiffnessBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModeShared])!
            let jointRotationBuffer = device.makeBuffer(length: count * MemoryLayout<simd_quatf>.stride, options: [.storageModeShared])!

            let childCount = childCountBuffer.contents().bindMemory(to: UInt8.self, capacity: count)
            let childIndex = childIndexBuffer.contents().bindMemory(to: UInt8.self, capacity: count)
            let parentId = parentIdBuffer.contents().bindMemory(to: UInt16.self, capacity: count)
            let climberCount = climberCountBuffer.contents().bindMemory(to: UInt8.self, capacity: count)
            let mass = massBuffer.contents().bindMemory(to: Float.self, capacity: count)
            let pivot = pivotBuffer.contents().bindMemory(to: simd_float3.self, capacity: count)
            let force = forceBuffer.contents().bindMemory(to: simd_float3.self, capacity: count)
            let torque = torqueBuffer.contents().bindMemory(to: simd_float3.self, capacity: count)
            let centerOfMass = centerOfMassBuffer.contents().bindMemory(to: simd_float3.self, capacity: count)
            let inertiaTensor = inertiaTensorBuffer.contents().bindMemory(to: simd_float3x3.self, capacity: count)
            let jointDamping = jointDampingBuffer.contents().bindMemory(to: Float.self, capacity: count)
            let jointStiffness = jointStiffnessBuffer.contents().bindMemory(to: Float.self, capacity: count)
            let jointRotation = jointRotationBuffer.contents().bindMemory(to: simd_quatf.self, capacity: count)

            // Step 3: Store data into the buffer
            func store(childIndex _childIndex: Int, parentId _parentId: Int, rigidBody: ArticulatedRigidBody, climbers: [ArticulatedRigidBody]) {
                assert(_parentId < UInt16.max)
                let id = index[rigidBody]!

                childCount[id] = UInt8(rigidBody.childJoints.count)
                childIndex[id] = UInt8(_childIndex)
                parentId[id] = UInt16(_parentId)
                climberCount[id] = UInt8(climbers.count)

                for rigidBody in climbers + [rigidBody] {
                    let id = index[rigidBody]!

                    mass[id] = rigidBody.mass
                    pivot[id] = rigidBody.pivot
                    force[id] = rigidBody.force
                    torque[id] = rigidBody.torque
                    centerOfMass[id] = rigidBody.centerOfMass
                    inertiaTensor[id] = rigidBody.inertiaTensor
                    if let parentJoint = rigidBody.parentJoint {
                        jointDamping[id] = parentJoint.damping
                        jointStiffness[id] = parentJoint.stiffness
                        jointRotation[id] = parentJoint.rotation
                    }
                }
            }

            for level in levels {
                for unitOfWork in level {
//                    let rigidBody = unitOfWork.rigidBody
                    store(childIndex: unitOfWork.childIndex, parentId: unitOfWork.parentId, rigidBody: unitOfWork.rigidBody, climbers: unitOfWork.climbers)
                }
            }
//            for rigidBody in allClimbers {
//                store(rigidBody: rigidBody)
//            }
//            id = index[root]!
//            store(id: id, rigidBody: root)

            // Step 4: Allocated the private buffers
            self.childCountBuffer = device.makeBuffer(length: childCountBuffer.length, options: [.storageModePrivate])!
            self.childIndexBuffer = device.makeBuffer(length: childIndexBuffer.length, options: [.storageModePrivate])!
            self.parentIdBuffer = device.makeBuffer(length: parentIdBuffer.length, options: [.storageModePrivate])!
            self.climberCountBuffer = device.makeBuffer(length: climberCountBuffer.length, options: [.storageModePrivate])!
            self.massBuffer = device.makeBuffer(length: massBuffer.length, options: [.storageModePrivate])!
            self.pivotBuffer = device.makeBuffer(length: pivotBuffer.length, options: [.storageModePrivate])!
            self.forceBuffer = device.makeBuffer(length: forceBuffer.length, options: [.storageModePrivate])!
            self.torqueBuffer = device.makeBuffer(length: torqueBuffer.length, options: [.storageModePrivate])!
            self.centerOfMassBuffer = device.makeBuffer(length: centerOfMassBuffer.length, options: [.storageModePrivate])!
            self.inertiaTensorBuffer = device.makeBuffer(length: inertiaTensorBuffer.length, options: [.storageModePrivate])!
            self.jointDampingBuffer = device.makeBuffer(length: jointDampingBuffer.length, options: [.storageModeShared])!
            self.jointStiffnessBuffer = device.makeBuffer(length: jointStiffnessBuffer.length, options: [.storageModeShared])!
            self.jointRotationBuffer = device.makeBuffer(length: jointRotationBuffer.length, options: [.storageModeShared])!

            // Step 5: Blit (copy) from the shared to private buffers
            let commandQueue = device.makeCommandQueue()!
            let commandBuffer = commandQueue.makeCommandBuffer()!
            let blitCommandEncoder = commandBuffer.makeBlitCommandEncoder()!

            blitCommandEncoder.copy(from: childCountBuffer, sourceOffset: 0, to: self.childCountBuffer, destinationOffset: 0, size: childCountBuffer.length)
            blitCommandEncoder.copy(from: childIndexBuffer, sourceOffset: 0, to: self.childIndexBuffer, destinationOffset: 0, size: childIndexBuffer.length)
            blitCommandEncoder.copy(from: parentIdBuffer, sourceOffset: 0, to: self.parentIdBuffer, destinationOffset: 0, size: parentIdBuffer.length)
            blitCommandEncoder.copy(from: climberCountBuffer, sourceOffset: 0, to: self.climberCountBuffer, destinationOffset: 0, size: climberCountBuffer.length)
            blitCommandEncoder.copy(from: massBuffer, sourceOffset: 0, to: self.massBuffer, destinationOffset: 0, size: massBuffer.length)
            blitCommandEncoder.copy(from: pivotBuffer, sourceOffset: 0, to: self.pivotBuffer, destinationOffset: 0, size: pivotBuffer.length)
            blitCommandEncoder.copy(from: forceBuffer, sourceOffset: 0, to: self.forceBuffer, destinationOffset: 0, size: forceBuffer.length)
            blitCommandEncoder.copy(from: torqueBuffer, sourceOffset: 0, to: self.torqueBuffer, destinationOffset: 0, size: torqueBuffer.length)
            blitCommandEncoder.copy(from: centerOfMassBuffer, sourceOffset: 0, to: self.centerOfMassBuffer, destinationOffset: 0, size: centerOfMassBuffer.length)
            blitCommandEncoder.copy(from: inertiaTensorBuffer, sourceOffset: 0, to: self.inertiaTensorBuffer, destinationOffset: 0, size: inertiaTensorBuffer.length)

            blitCommandEncoder.endEncoding()
            commandBuffer.commit()
            commandBuffer.waitUntilCompleted()
        }
    }

    class CompositeBodies {
        let count: Int
        let ranges: [Range<Int>]
        let massBuffer, forceBuffer, torqueBuffer, pivotBuffer, centerOfMassBuffer, inertiaTensorBuffer: MTLBuffer

        init(device: MTLDevice, count: Int, ranges: [Range<Int>]) {
            self.count = count
            self.ranges = ranges

            self.massBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModePrivate])!
            self.forceBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.torqueBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.pivotBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.centerOfMassBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.inertiaTensorBuffer = device.makeBuffer(length: count * MemoryLayout<matrix_float3x3>.stride, options: [.storageModePrivate])!
        }

        subscript(id: Int) -> CompositeBody? {
//            if ranges[0].contains(id) {
//                return nil
//            } else {
                return CompositeBody(
                    mass: mass[id],
                    inertiaTensor: inertiaTensor[id],
                    force: force[id],
                    torque: torque[id],
                    centerOfMass: centerOfMass[id])
//            }
        }

        var mass: UnsafeMutablePointer<Float> {
            return massBuffer.contents().bindMemory(to: Float.self, capacity: count)
        }
        var force: UnsafeMutablePointer<simd_float3> {
            return forceBuffer.contents().bindMemory(to: simd_float3.self, capacity: count)
        }
        var torque: UnsafeMutablePointer<simd_float3> {
            return torqueBuffer.contents().bindMemory(to: simd_float3.self, capacity: count)
        }
        var centerOfMass: UnsafeMutablePointer<simd_float3> {
            return centerOfMassBuffer.contents().bindMemory(to: simd_float3.self, capacity: count)
        }
        var inertiaTensor: UnsafeMutablePointer<simd_float3x3> {
            inertiaTensorBuffer.contents().bindMemory(to: simd_float3x3.self, capacity: count)
        }

    }

    // FIXME move into rigid body
    class Joints {
        let thetaBuffer: MTLBuffer

        init(device: MTLDevice, count: Int) {
            self.thetaBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3x3>.stride, options: [.storageModePrivate])!
        }
    }

    class PhysicsFields {}

    let rigidBodies: RigidBodies
    let compositeBodies: CompositeBodies
    let joints: Joints

    init(device: MTLDevice, root: ArticulatedRigidBody) {
        self.rigidBodies = RigidBodies(device: device, root: root)
        self.compositeBodies = CompositeBodies(device: device, count: rigidBodies.count, ranges: rigidBodies.ranges)
        self.joints = Joints(device: device, count: rigidBodies.count)
    }
}
