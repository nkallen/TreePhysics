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
    let count: Int
    let maxChars: Int
    let label: String

    init(device: MTLDevice, count: Int = 16, maxChars: Int = 2048, label: String = "") {
        self.maxChars = maxChars
        self.stringBuffer = device.makeBuffer(length: maxChars * count, options: [.storageModeShared])!
        self.count = count
        self.label = label
    }

    func encode(commandEncoder: MTLComputeCommandEncoder) {
        commandEncoder.setBuffer(stringBuffer, offset: 0, index: BufferIndex.debugString.rawValue)
    }

    func wrap(_ commandBuffer: MTLCommandBuffer) -> MTLCommandBuffer {
        return KernelDebuggerCommandBufferProxy(commandBuffer, debugger: self)
    }

    var strings: [String] {
        var pointer = UnsafeMutableRawPointer(self.stringBuffer.contents()).bindMemory(to: CChar.self, capacity: maxChars * count)
        var result: [String] = []
        for _ in 0..<count {
            result.append(String(cString: pointer))
            pointer = pointer.advanced(by: maxChars)
        }

        return result
    }

    func print(count: Int? = nil) {
        let count = count ?? self.count
        for i in 0..<count {
            let string = strings[i]
            let lines = string.split { $0.isNewline }
            for line in lines {
                Swift.print("\(label)[\(i)]: ", line)
            }
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
