// Generated using Sourcery 0.17.0 — https://github.com/krzysztofzablocki/Sourcery
// DO NOT EDIT

protocol Allocator {
    associatedtype Pointee
    static func allocate(capacity count: Int) -> UnsafeMutablePointer<Pointee>
    func assign(from source: UnsafePointer<Pointee>, count: Int)
}

struct RigidBody2Allocator: Allocator {
    typealias Pointee RigidBody2

    let parentIdBuffer: MTLBuffer
    let childCountBuffer: MTLBuffer
    let firstChildBuffer: MTLBuffer
    let childIndexBuffer: MTLBuffer
    let massBuffer: MTLBuffer
    let pivotBuffer: MTLBuffer
    let centerOfMassBuffer: MTLBuffer

    let parentId: UnsafeMutablePointer<Int>
    let childCount: UnsafeMutablePointer<Int>
    let firstChild: UnsafeMutablePointer<Int>
    let childIndex: UnsafeMutablePointer<Int>
    let mass: UnsafeMutablePointer<Float>
    let pivot: UnsafeMutablePointer<simd_float3>
    let centerOfMass: UnsafeMutablePointer<simd_float3>

    static func allocate(capacity count: Int, with device: MTLDevice, options: MTLResourceOptions = []) -> Self {
        self.parentIdBuffer = device.makeBuffer(length: count * MemoryLayout<Int>.stride, options: options)!
        self.childCountBuffer = device.makeBuffer(length: count * MemoryLayout<Int>.stride, options: options)!
        self.firstChildBuffer = device.makeBuffer(length: count * MemoryLayout<Int>.stride, options: options)!
        self.childIndexBuffer = device.makeBuffer(length: count * MemoryLayout<Int>.stride, options: options)!
        self.massBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: options)!
        self.pivotBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: options)!
        self.centerOfMassBuffer = device.makeBuffer(length: count * MemoryLayout<simd_float3>.stride, options: options)!
    }

    init(
        parentIdBuffer: MTLBuffer,
        childCountBuffer: MTLBuffer,
        firstChildBuffer: MTLBuffer,
        childIndexBuffer: MTLBuffer,
        massBuffer: MTLBuffer,
        pivotBuffer: MTLBuffer,
        centerOfMassBuffer: MTLBuffer,
    ) {
        self.parentIdBuffer = parentIdBuffer
        self.childCountBuffer = childCountBuffer
        self.firstChildBuffer = firstChildBuffer
        self.childIndexBuffer = childIndexBuffer
        self.massBuffer = massBuffer
        self.pivotBuffer = pivotBuffer
        self.centerOfMassBuffer = centerOfMassBuffer

        self.parentIdBuffer.label = "RigidBody2.parentId"
        self.childCountBuffer.label = "RigidBody2.childCount"
        self.firstChildBuffer.label = "RigidBody2.firstChild"
        self.childIndexBuffer.label = "RigidBody2.childIndex"
        self.massBuffer.label = "RigidBody2.mass"
        self.pivotBuffer.label = "RigidBody2.pivot"
        self.centerOfMassBuffer.label = "RigidBody2.centerOfMass"

        self.parentId = parentIdBuffer.contents().bindMemory(to: Int.self, capacity: count)
        self.childCount = childCountBuffer.contents().bindMemory(to: Int.self, capacity: count)
        self.firstChild = firstChildBuffer.contents().bindMemory(to: Int.self, capacity: count)
        self.childIndex = childIndexBuffer.contents().bindMemory(to: Int.self, capacity: count)
        self.mass = massBuffer.contents().bindMemory(to: Float.self, capacity: count)
        self.pivot = pivotBuffer.contents().bindMemory(to: simd_float3.self, capacity: count)
        self.centerOfMass = centerOfMassBuffer.contents().bindMemory(to: simd_float3.self, capacity: count)
    }

    subscript(id: Int) -> RigidBody2 {
        get {
            return RigidBody2(
                parentId: self.parentId[id],
                childCount: self.childCount[id],
                firstChild: self.firstChild[id],
                childIndex: self.childIndex[id],
                mass: self.mass[id],
                pivot: self.pivot[id],
                centerOfMass: self.centerOfMass[id],
            )
        }

        nonmutating set {

        }
    }

    func assign(from other: Self, with: blitCommandEncoder: MTLBlitCommandEncoder) {
        blitCommandEncoder.copy(from: other.parentIdBuffer, sourceOffset: 0, to: self.parentIdBuffer, destinationOffset: 0, size: other.parentIdBuffer.length)
        blitCommandEncoder.copy(from: other.childCountBuffer, sourceOffset: 0, to: self.childCountBuffer, destinationOffset: 0, size: other.childCountBuffer.length)
        blitCommandEncoder.copy(from: other.firstChildBuffer, sourceOffset: 0, to: self.firstChildBuffer, destinationOffset: 0, size: other.firstChildBuffer.length)
        blitCommandEncoder.copy(from: other.childIndexBuffer, sourceOffset: 0, to: self.childIndexBuffer, destinationOffset: 0, size: other.childIndexBuffer.length)
        blitCommandEncoder.copy(from: other.massBuffer, sourceOffset: 0, to: self.massBuffer, destinationOffset: 0, size: other.massBuffer.length)
        blitCommandEncoder.copy(from: other.pivotBuffer, sourceOffset: 0, to: self.pivotBuffer, destinationOffset: 0, size: other.pivotBuffer.length)
        blitCommandEncoder.copy(from: other.centerOfMassBuffer, sourceOffset: 0, to: self.centerOfMassBuffer, destinationOffset: 0, size: other.centerOfMassBuffer.length)
    }
}
