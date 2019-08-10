import Foundation
import MetalKit

let maxBuffersInFlight = 3

class MetalKernel {
    let device: MTLDevice
    let computePipelineState: MTLComputePipelineState

    init(device: MTLDevice, name: String) {
        self.device = device
        let library = device.makeDefaultLibrary()!
        let function = library.makeFunction(name: name)!
        self.computePipelineState = try! device.makeComputePipelineState(function: function)
    }

    init(device: MTLDevice, function: MTLFunction) {
        self.device = device
        self.computePipelineState = try! device.makeComputePipelineState(function: function)
    }
}

class KernelDebugger {
    let rigidBodiesBuffer: MTLBuffer
    let compositeBodiesBuffer: MTLBuffer
    let jointsBuffer: MTLBuffer
    let floatsBuffer: MTLBuffer
    let float3sBuffer: MTLBuffer
    let float3x3sBuffer: MTLBuffer

    init(device: MTLDevice) {
        self.rigidBodiesBuffer = device.makeBuffer(length: 1024, options: [.storageModeShared])!
        self.compositeBodiesBuffer = device.makeBuffer(length: 1024, options: [.storageModeShared])!
        self.jointsBuffer = device.makeBuffer(length: 1024, options: [.storageModeShared])!
        self.floatsBuffer = device.makeBuffer(length: 1024, options: [.storageModeShared])!
        self.float3sBuffer = device.makeBuffer(length: 1024, options: [.storageModeShared])!
        self.float3x3sBuffer = device.makeBuffer(length: 1024, options: [.storageModeShared])!
    }

    func encode(commandEncoder: MTLComputeCommandEncoder) {
        commandEncoder.setBuffer(rigidBodiesBuffer, offset: 0, index: BufferIndex.debugRigidBody.rawValue)
        commandEncoder.setBuffer(compositeBodiesBuffer, offset: 0, index: BufferIndex.debugCompositeBody.rawValue)
        commandEncoder.setBuffer(jointsBuffer, offset: 0, index: BufferIndex.debugJoint.rawValue)
        commandEncoder.setBuffer(floatsBuffer, offset: 0, index: BufferIndex.debugFloat.rawValue)
        commandEncoder.setBuffer(float3sBuffer, offset: 0, index: BufferIndex.debugFloat3.rawValue)
        commandEncoder.setBuffer(float3x3sBuffer, offset: 0, index: BufferIndex.debugFloat3x3.rawValue)
    }

    func wrap(commandBuffer: MTLCommandBuffer) -> MTLCommandBuffer {
        return KernelDebuggerCommandBufferProxy(commandBuffer, debugger: self)
    }

    var rigidBodies: UnsafeMutablePointer<RigidBodyStruct> {
        return UnsafeMutableRawPointer(self.rigidBodiesBuffer.contents()).bindMemory(to: RigidBodyStruct.self, capacity: 10)
    }

    var compositeBodies: UnsafeMutablePointer<CompositeBodyStruct> {
        return UnsafeMutableRawPointer(self.compositeBodiesBuffer.contents()).bindMemory(to: CompositeBodyStruct.self, capacity: 10)
    }

    var joints: UnsafeMutablePointer<JointStruct> {
        return UnsafeMutableRawPointer(self.jointsBuffer.contents()).bindMemory(to: JointStruct.self, capacity: 10)
    }
    
    var floats: UnsafeMutablePointer<Float> {
        return UnsafeMutableRawPointer(self.floatsBuffer.contents()).bindMemory(to: Float.self, capacity: 10)
    }

    var float3s: UnsafeMutablePointer<float3> {
        return UnsafeMutableRawPointer(self.float3sBuffer.contents()).bindMemory(to: float3.self, capacity: 10)
    }

    var float3x3s: UnsafeMutablePointer<float3x3> {
        return UnsafeMutableRawPointer(self.float3x3sBuffer.contents()).bindMemory(to: float3x3.self, capacity: 10)
    }
}

class KernelDebuggerCommandBufferProxy: MTLCommandBufferProxy {
    let debugger: KernelDebugger

    init(_ underlying: MTLCommandBuffer, debugger: KernelDebugger) {
        self.debugger = debugger
        super.init(underlying)
    }

    override func makeComputeCommandEncoder() -> MTLComputeCommandEncoder? {
        guard let commandEncoder = super.makeComputeCommandEncoder() else { return nil }
        return KernelDebuggerComputeCommandEncoderProxy(commandEncoder, debugger: debugger)
    }
}

class KernelDebuggerComputeCommandEncoderProxy: MTLComputeCommandEncoderProxy {
    let debugger: KernelDebugger

    init(_ underlying: MTLComputeCommandEncoder, debugger: KernelDebugger) {
        self.debugger = debugger
        super.init(underlying)
    }

    override func setComputePipelineState(_ state: MTLComputePipelineState) {
        super.setComputePipelineState(state)
        debugger.encode(commandEncoder: underlying)
    }
}
