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
    let stringBuffer: MTLBuffer

    init(device: MTLDevice) {
        self.stringBuffer = device.makeBuffer(length: 1024 * 16, options: [.storageModeShared])!
    }

    func encode(commandEncoder: MTLComputeCommandEncoder) {
        commandEncoder.setBuffer(stringBuffer, offset: 0, index: BufferIndex.debugString.rawValue)
    }

    func wrap(commandBuffer: MTLCommandBuffer) -> MTLCommandBuffer {
        return KernelDebuggerCommandBufferProxy(commandBuffer, debugger: self)
    }

    var strings: [String] {
        var pointer = UnsafeMutableRawPointer(self.stringBuffer.contents()).bindMemory(to: CChar.self, capacity: 1024 * 16)
        var result: [String] = []
        for _ in 0..<16 {
            result.append(String(cString: pointer))
            pointer = pointer.advanced(by: 1024)
        }

        return result
    }

    func print() {
        for string in strings {
            Swift.print(string)
        }
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
