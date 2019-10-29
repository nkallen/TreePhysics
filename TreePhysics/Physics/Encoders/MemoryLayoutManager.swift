import Foundation
import simd
import MetalKit

class MemoryLayoutManager {
    class RigidBodies {
        let ranges: [Range<Int>]
        let count: Int
        let childInfoBuffer, climberCountBuffer, firstClimberIdBuffer, massBuffer, pivotBuffer, forceBuffer, torqueBuffer, centerOfMassBuffer, inertiaTensorBuffer, jointDampingBuffer, jointStiffnessBuffer, jointRotationBuffer: MTLBuffer

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
            let jointDampingBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModeShared])!
            let jointStiffnessBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModeShared])!
            let jointRotationBuffer = device.makeBuffer(length: count * MemoryLayout<simd_quatf>.stride, options: [.storageModeShared])!

            let childInfo = childInfoBuffer.contents().bindMemory(to: simd_int2.self, capacity: count)
            let climberCount = climberCountBuffer.contents().bindMemory(to: UInt16.self, capacity: count)
            let firstClimberId = firstClimberIdBuffer.contents().bindMemory(to: Int32.self, capacity: count)
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
            self.childInfoBuffer = device.makeBuffer(length: childInfoBuffer.length, options: [.storageModePrivate])!
            self.climberCountBuffer = device.makeBuffer(length: climberCountBuffer.length, options: [.storageModePrivate])!
            self.firstClimberIdBuffer = device.makeBuffer(length: firstClimberIdBuffer.length, options: [.storageModePrivate])!
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

//            self.mass = massBuffer.contents().bindMemory(to: Float.self, capacity: count)
//            self.force = forceBuffer.contents().bindMemory(to: simd_float3.self, capacity: count)
//            self.torque = torqueBuffer.contents().bindMemory(to: simd_float3.self, capacity: count)
//            self.centerOfMass = centerOfMassBuffer.contents().bindMemory(to: simd_float3.self, capacity: count)
//            self.inertiaTensor = inertiaTensorBuffer.contents().bindMemory(to: simd_float3x3.self, capacity: count)
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
