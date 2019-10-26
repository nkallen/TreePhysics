import Foundation
import simd
import MetalKit

class MemoryLayoutManager {
    class RigidBodies {
        let ranges: [Range<Int>]
        let count: Int
        let childCount, firstChildId, mass, pivot, force, torque, centerOfMass, inertiaTensor: MTLBuffer

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

            // Step 2: Allocate the buffers
            self.count = offset + allClimbers.count + 1 // +1 for root
            self.childCount = device.makeBuffer(length: count * MemoryLayout<UInt16>.stride, options: [.storageModeShared])!
            self.firstChildId = device.makeBuffer(length: count * MemoryLayout<Int>.stride, options: [.storageModeShared])!
            self.mass = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModeShared])!
            self.pivot = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            self.force = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            self.torque = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            self.centerOfMass = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            self.inertiaTensor = device.makeBuffer(length: count * MemoryLayout<matrix_float3x3>.stride, options: [.storageModeShared])!

            // Step 3: Bind the buffers to swift types
            let childCount = UnsafeMutableRawPointer(self.childCount.contents())!.bindMemory(to: UInt16.self, capacity: count)
            let firstChildId = UnsafeMutableRawPointer(self.firstChildId.contents())!.bindMemory(to: Int.self, capacity: count)
            let mass = UnsafeMutableRawPointer(self.mass.contents())!.bindMemory(to: Float.self, capacity: count)
            let pivot = UnsafeMutableRawPointer(self.pivot.contents())!.bindMemory(to: simd_float3.self, capacity: count)
            let force = UnsafeMutableRawPointer(self.force.contents())!.bindMemory(to: simd_float3.self, capacity: count)
            let torque = UnsafeMutableRawPointer(self.torque.contents())!.bindMemory(to: simd_float3.self, capacity: count)
            let centerOfMass = UnsafeMutableRawPointer(self.centerOfMass.contents())!.bindMemory(to: simd_float3.self, capacity: count)
            let inertiaTensor = UnsafeMutableRawPointer(self.inertiaTensor.contents())!.bindMemory(to: simd_float3x3.self, capacity: count)

            // Step 4: Store data into the buffer
            for level in levels {
                for unitOfWork in level {
                    let id = index[unitOfWork.rigidBody]!
                    childCount[id] = UInt16(unitOfWork.rigidBody.childJoints.count)
                    if let childJoint = unitOfWork.rigidBody.childJoints.first {
                        firstChildId[id] = index[childJoint.childRigidBody]! // FIXME set unordered
                    }

                }
            }
            for rigidBody in allClimbers {
                let id = index[rigidBody]!
                childCount[id] = UInt16(rigidBody.childJoints.count)
                firstChildId[id] = index[rigidBody.childJoints.first!.childRigidBody]!
            }
            id = index[root]!
            childCount[id] = UInt16(root.childJoints.count)
            firstChildId[id] = index[root.childJoints.first!.childRigidBody]!
        }
    }

    class CompositeBody {
        let mass, force, torque, centerOfMass, inertiaTensor: MTLBuffer

        init(device: MTLDevice, count: Int) {
            self.mass = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModePrivate])!
            self.force = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.torque = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.centerOfMass = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.inertiaTensor = device.makeBuffer(length: count * MemoryLayout<matrix_float3x3>.stride, options: [.storageModePrivate])!
        }
    }

    let rigidBody: RigidBodies
    let compositeBody: CompositeBody

    init(device: MTLDevice, root: ArticulatedRigidBody) {
        self.rigidBody = RigidBodies(device: device, root: root)
        self.compositeBody = CompositeBody(device: device, count: rigidBody.count)
    }
}
