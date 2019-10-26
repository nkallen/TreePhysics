import Foundation
import simd
import MetalKit

class MemoryLayoutManager {
    class RigidBodies {
        let ranges: [Range<Int>]
        let count: Int
        let childCountBuffer, firstChildIdBuffer, climberCountBuffer, firstClimberIdBuffer, massBuffer, pivotBuffer, forceBuffer, torqueBuffer, centerOfMassBuffer, inertiaTensorBuffer: MTLBuffer
        let childCount: UnsafeMutablePointer<UInt16>, firstChildId: UnsafeMutablePointer<Int32>
        let climberCount: UnsafeMutablePointer<UInt16>, firstClimberId: UnsafeMutablePointer<Int32>
        let mass: UnsafeMutablePointer<Float>
        let pivot, force, torque, centerOfMass: UnsafeMutablePointer<simd_float3>
        let inertiaTensor: UnsafeMutablePointer<simd_float3x3>

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
            self.childCountBuffer = device.makeBuffer(length: count * MemoryLayout<UInt16>.stride, options: [.storageModeShared])!
            self.firstChildIdBuffer = device.makeBuffer(length: count * MemoryLayout<Int32>.stride, options: [.storageModeShared])!
            self.climberCountBuffer = device.makeBuffer(length: count * MemoryLayout<UInt16>.stride, options: [.storageModeShared])!
            self.firstClimberIdBuffer = device.makeBuffer(length: count * MemoryLayout<Int32>.stride, options: [.storageModeShared])!
            self.massBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModeShared])!
            self.pivotBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            self.forceBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            self.torqueBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            self.centerOfMassBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            self.inertiaTensorBuffer = device.makeBuffer(length: count * MemoryLayout<matrix_float3x3>.stride, options: [.storageModeShared])!

            self.childCount = UnsafeMutableRawPointer(childCountBuffer.contents())!.bindMemory(to: UInt16.self, capacity: count)
            self.firstChildId = UnsafeMutableRawPointer(firstChildIdBuffer.contents())!.bindMemory(to: Int32.self, capacity: count)
            self.climberCount = UnsafeMutableRawPointer(climberCountBuffer.contents())!.bindMemory(to: UInt16.self, capacity: count)
            self.firstClimberId = UnsafeMutableRawPointer(firstClimberIdBuffer.contents())!.bindMemory(to: Int32.self, capacity: count)
            self.mass = UnsafeMutableRawPointer(massBuffer.contents())!.bindMemory(to: Float.self, capacity: count)
            self.pivot = UnsafeMutableRawPointer(pivotBuffer.contents())!.bindMemory(to: simd_float3.self, capacity: count)
            self.force = UnsafeMutableRawPointer(forceBuffer.contents())!.bindMemory(to: simd_float3.self, capacity: count)
            self.torque = UnsafeMutableRawPointer(torqueBuffer.contents())!.bindMemory(to: simd_float3.self, capacity: count)
            self.centerOfMass = UnsafeMutableRawPointer(centerOfMassBuffer.contents())!.bindMemory(to: simd_float3.self, capacity: count)
            self.inertiaTensor = UnsafeMutableRawPointer(inertiaTensorBuffer.contents())!.bindMemory(to: simd_float3x3.self, capacity: count)

            // Step 3: Store data into the buffer
            for level in levels {
                for unitOfWork in level {
                    let rigidBody = unitOfWork.rigidBody
                    let id = index[rigidBody]!
                    store(id: id, rigidBody: rigidBody, climbers: unitOfWork.climbers, index: index)
                }
            }
            for rigidBody in allClimbers {
                let id = index[rigidBody]!
                store(id: id, rigidBody: rigidBody, index: index)
            }
            id = index[root]!
            store(id: id, rigidBody: root, index: index)
        }

        private func store(id: Int, rigidBody: ArticulatedRigidBody, climbers: [ArticulatedRigidBody] = [], index: [RigidBody:Int] = [:]) {
            childCount[id] = UInt16(rigidBody.childJoints.count)
            if let childJoint = rigidBody.childJoints.first {
                firstChildId[id] = Int32(index[childJoint.childRigidBody]!) // FIXME set unordered
            }
            climberCount[id] = UInt16(climbers.count)
            firstClimberId[id] = Int32(climbers.first.map { index[$0]! } ?? 0)
            mass[id] = rigidBody.mass
        }

    }

    class CompositeBodies {
        let count: Int
        let massBuffer, forceBuffer, torqueBuffer, centerOfMassBuffer, inertiaTensorBuffer: MTLBuffer
        let mass: UnsafeMutablePointer<Float>
        let force, torque, centerOfMass: UnsafeMutablePointer<simd_float3>
        let inertiaTensor: UnsafeMutablePointer<simd_float3x3>

        init(device: MTLDevice, count: Int) {
            self.count = count

            self.massBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModePrivate])!
            self.forceBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.torqueBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.centerOfMassBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.inertiaTensorBuffer = device.makeBuffer(length: count * MemoryLayout<matrix_float3x3>.stride, options: [.storageModePrivate])!

            self.mass = UnsafeMutableRawPointer(massBuffer.contents())!.bindMemory(to: Float.self, capacity: count)
            self.force = UnsafeMutableRawPointer(forceBuffer.contents())!.bindMemory(to: simd_float3.self, capacity: count)
            self.torque = UnsafeMutableRawPointer(torqueBuffer.contents())!.bindMemory(to: simd_float3.self, capacity: count)
            self.centerOfMass = UnsafeMutableRawPointer(centerOfMassBuffer.contents())!.bindMemory(to: simd_float3.self, capacity: count)
            self.inertiaTensor = UnsafeMutableRawPointer(inertiaTensorBuffer.contents())!.bindMemory(to: simd_float3x3.self, capacity: count)
        }

        subscript(id: Int) -> CompositeBody {
            return CompositeBody(
                mass: mass[id],
                inertiaTensor: inertiaTensor[id],
                force: force[id],
                torque: torque[id],
                centerOfMass: centerOfMass[id])
        }
    }

    let rigidBodies: RigidBodies
    let compositeBodies: CompositeBodies

    init(device: MTLDevice, root: ArticulatedRigidBody) {
        self.rigidBodies = RigidBodies(device: device, root: root)
        self.compositeBodies = CompositeBodies(device: device, count: rigidBodies.count)
    }
}
