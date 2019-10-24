import Foundation
import MetalKit
import ShaderTypes

class KernelDebugger {
    let stringBuffer: MTLBuffer
    var count: Int? = nil
    let label: String

    init(device: MTLDevice, length: Int = 1024 * 1024, label: String = "") {
        self.stringBuffer = device.makeBuffer(length: length, options: [.storageModeShared])!
        self.label = label
    }

    func encode(commandEncoder: MTLComputeCommandEncoder) {
        commandEncoder.setBuffer(stringBuffer, offset: 0, index: BufferIndex.debug.rawValue)
        var length = stringBuffer.allocatedSize
        commandEncoder.setBytes(&length, length: MemoryLayout<Int>.stride, index: BufferIndex.debugLength.rawValue)
    }

    func wrap(_ commandBuffer: MTLCommandBuffer) -> MTLCommandBuffer {
        return KernelDebuggerCommandBufferProxy(commandBuffer, debugger: self)
    }

    var strings: [String] {
        guard let count = count else { return [] }

        var pointer = UnsafeMutableRawPointer(stringBuffer.contents()).bindMemory(to: CChar.self, capacity: stringBuffer.allocatedSize)
        var result: [String] = []
        for _ in 0..<count {
            result.append(String(cString: pointer))
            pointer = pointer.advanced(by: stringBuffer.allocatedSize / count)
        }

        return result
    }

    func print() {
        guard let count = count else { return }

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

    override func dispatchThreads(_ threadsPerGrid: MTLSize, threadsPerThreadgroup: MTLSize) {
        debugger.count = threadsPerGrid.width * threadsPerGrid.height * threadsPerGrid.depth
        underlying.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
    }
}
