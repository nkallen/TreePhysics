import Foundation
import simd
import MetalKit

class MemoryLayoutManager {
    struct RigidBody {
        let count: Int
        let childCount, mass, pivot, force, torque, centerOfMass, inertiaTensor: MTLBuffer

        init(device: MTLDevice, count: Int) {
            self.count = count

            self.childCount = device.makeBuffer(length: count * MemoryLayout<UInt16>.stride, options: [.storageModeShared])!
            self.mass = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModeShared])!
            self.pivot = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            self.force = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            self.torque = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            self.centerOfMass = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModeShared])!
            self.inertiaTensor = device.makeBuffer(length: count * MemoryLayout<matrix_float3x3>.stride, options: [.storageModeShared])!
        }

        // FIXME use a regular method
        subscript(index: Int) -> ArticulatedRigidBody {
            set(newValue) {
                UnsafeMutableRawPointer(childCount.contents())!.bindMemory(to: UInt16.self, capacity: count)[index] = UInt16(newValue.childJoints.count)
            }

            get {
                fatalError()
            }
        }
    }
    struct CompositeBody {
        let mass, force, torque, centerOfMass, inertiaTensor: MTLBuffer

        init(device: MTLDevice, count: Int) {
            self.mass = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModePrivate])!
            self.force = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.torque = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.centerOfMass = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: [.storageModePrivate])!
            self.inertiaTensor = device.makeBuffer(length: count * MemoryLayout<matrix_float3x3>.stride, options: [.storageModePrivate])!
        }
    }

    var rigidBody: RigidBody
    var compositeBody: CompositeBody

    init(device: MTLDevice, count: Int) {
        self.rigidBody = RigidBody(device: device, count: count)
        self.compositeBody = CompositeBody(device: device, count: count)
    }
}
