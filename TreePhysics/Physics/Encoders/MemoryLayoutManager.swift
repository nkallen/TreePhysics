import Foundation
import simd
import MetalKit

class MemoryLayoutManager {
    class RigidBodies {
        let ranges: [Range<Int>]
        let count: Int
        let childInfoBuffer, climberCountBuffer, firstClimberIdBuffer, massBuffer, pivotBuffer, forceBuffer, torqueBuffer, centerOfMassBuffer, inertiaTensorBuffer: MTLBuffer

        init(device: MTLDevice, root: ArticulatedRigidBody) {
            var rangesOfWork: [Range<Int>] = []
            let levels = root.levels()
            var offset = 0
            var id = 0
            var index = [RigidBody:Int]()
            var allClimbers: [ArticulatedRigidBody] = []

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
            self.ranges = rangesOfWork
            for rigidBody in allClimbers {
                index[rigidBody] = id
                id += 1
            }
            index[root] = id

            // Step 2: Allocate the shared buffers
            self.count = offset + allClimbers.count + 1 // +1 for root

            let childInfoBuffer = device.makeBuffer(length: count * MemoryLayout<simd_int2>.stride, options: [.storageModeShared])!
            let climberCountBuffer = device.makeBuffer(length: count * MemoryLayout<UInt16>.stride, options: [.storageModeShared])!
            let firstClimberIdBuffer = device.makeBuffer(length: count * MemoryLayout<Int32>.stride, options: [.storageModeShared])!
            let massBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModeShared])!
            let pivotBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            let forceBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            let torqueBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            let centerOfMassBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            let inertiaTensorBuffer = device.makeBuffer(length: count * MemoryLayout<matrix_float3x3>.stride, options: [.storageModeShared])!

            let childInfo = UnsafeMutableRawPointer(childInfoBuffer.contents())!.bindMemory(to: simd_int2.self, capacity: count)
            let climberCount = UnsafeMutableRawPointer(climberCountBuffer.contents())!.bindMemory(to: UInt16.self, capacity: count)
            let firstClimberId = UnsafeMutableRawPointer(firstClimberIdBuffer.contents())!.bindMemory(to: Int32.self, capacity: count)
            let mass = UnsafeMutableRawPointer(massBuffer.contents())!.bindMemory(to: Float.self, capacity: count)
            let pivot = UnsafeMutableRawPointer(pivotBuffer.contents())!.bindMemory(to: simd_float3.self, capacity: count)
            let force = UnsafeMutableRawPointer(forceBuffer.contents())!.bindMemory(to: simd_float3.self, capacity: count)
            let torque = UnsafeMutableRawPointer(torqueBuffer.contents())!.bindMemory(to: simd_float3.self, capacity: count)
            let centerOfMass = UnsafeMutableRawPointer(centerOfMassBuffer.contents())!.bindMemory(to: simd_float3.self, capacity: count)
            let inertiaTensor = UnsafeMutableRawPointer(inertiaTensorBuffer.contents())!.bindMemory(to: simd_float3x3.self, capacity: count)

            // Step 3: Store data into the buffer
            func store(id: Int, rigidBody: ArticulatedRigidBody, climbers: [ArticulatedRigidBody] = []) {
                if let childJoint = rigidBody.childJoints.first {
                    let firstChildId = index[childJoint.childRigidBody]!  // FIXME set unordered
                    childInfo[id] = simd_int2(Int32(rigidBody.childJoints.count), Int32(firstChildId))
                } else {
                    childInfo[id] = simd_int2(0, -1)
                }
                climberCount[id] = UInt16(climbers.count)
                firstClimberId[id] = Int32(climbers.first.map { index[$0]! } ?? 0)
                mass[id] = rigidBody.mass
            }

            for level in levels {
                for unitOfWork in level {
                    let rigidBody = unitOfWork.rigidBody
                    let id = index[rigidBody]!
                    store(id: id, rigidBody: rigidBody, climbers: unitOfWork.climbers)
                }
            }
            for rigidBody in allClimbers {
                let id = index[rigidBody]!
                store(id: id, rigidBody: rigidBody)
            }
            id = index[root]!
            store(id: id, rigidBody: root)

            // Step 4: Allocated the private buffers
            self.childInfoBuffer = device.makeBuffer(length: count * MemoryLayout<simd_int2>.stride, options: [.storageModePrivate])!
            self.climberCountBuffer = device.makeBuffer(length: count * MemoryLayout<UInt16>.stride, options: [.storageModePrivate])!
            self.firstClimberIdBuffer = device.makeBuffer(length: count * MemoryLayout<Int32>.stride, options: [.storageModePrivate])!
            self.massBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModePrivate])!
            self.pivotBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.forceBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.torqueBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.centerOfMassBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.inertiaTensorBuffer = device.makeBuffer(length: count * MemoryLayout<matrix_float3x3>.stride, options: [.storageModePrivate])!

            // Step 5: Blit (copy) from the shared to private buffers
            let commandQueue = device.makeCommandQueue()!
            let commandBuffer = commandQueue.makeCommandBuffer()!
            let blitCommandEncoder = commandBuffer.makeBlitCommandEncoder()!

            blitCommandEncoder.copy(from: childInfoBuffer, sourceOffset: 0, to: self.childInfoBuffer, destinationOffset: 0, size: childInfoBuffer.length)
            blitCommandEncoder.copy(from: climberCountBuffer, sourceOffset: 0, to: self.climberCountBuffer, destinationOffset: 0, size: climberCountBuffer.length)
            blitCommandEncoder.copy(from: firstClimberIdBuffer, sourceOffset: 0, to: self.firstClimberIdBuffer, destinationOffset: 0, size: firstClimberIdBuffer.length)
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
        let massBuffer, forceBuffer, torqueBuffer, centerOfMassBuffer, inertiaTensorBuffer: MTLBuffer
//        let mass: UnsafeMutablePointer<Float>
//        let force, torque, centerOfMass: UnsafeMutablePointer<simd_float3>
//        let inertiaTensor: UnsafeMutablePointer<simd_float3x3>

        init(device: MTLDevice, count: Int, ranges: [Range<Int>]) {
            self.count = count
            self.ranges = ranges

            self.massBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModePrivate])!
            self.forceBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.torqueBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.centerOfMassBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.inertiaTensorBuffer = device.makeBuffer(length: count * MemoryLayout<matrix_float3x3>.stride, options: [.storageModePrivate])!

//            self.mass = UnsafeMutableRawPointer(massBuffer.contents())!.bindMemory(to: Float.self, capacity: count)
//            self.force = UnsafeMutableRawPointer(forceBuffer.contents())!.bindMemory(to: simd_float3.self, capacity: count)
//            self.torque = UnsafeMutableRawPointer(torqueBuffer.contents())!.bindMemory(to: simd_float3.self, capacity: count)
//            self.centerOfMass = UnsafeMutableRawPointer(centerOfMassBuffer.contents())!.bindMemory(to: simd_float3.self, capacity: count)
//            self.inertiaTensor = UnsafeMutableRawPointer(inertiaTensorBuffer.contents())!.bindMemory(to: simd_float3x3.self, capacity: count)
        }

//        subscript(id: Int) -> CompositeBody? {
//            if ranges[0].contains(id) {
//                return nil
//            } else {
//                return CompositeBody(
//                    mass: mass[id],
//                    inertiaTensor: inertiaTensor[id],
//                    force: force[id],
//                    torque: torque[id],
//                    centerOfMass: centerOfMass[id])
//            }
//        }
    }

    class Joints {
        let thetaBuffer: MTLBuffer

        init(device: MTLDevice, count: Int) {
            self.thetaBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3x3>.stride, options: [.storageModePrivate])!
        }
    }

    let rigidBodies: RigidBodies
    let compositeBodies: CompositeBodies
    let joints: Joints

    init(device: MTLDevice, root: ArticulatedRigidBody) {
        self.rigidBodies = RigidBodies(device: device, root: root)
        self.compositeBodies = CompositeBodies(device: device, count: rigidBodies.count, ranges: rigidBodies.ranges)
        self.joints = Joints(device: device, count: rigidBodies.count)
    }
}
