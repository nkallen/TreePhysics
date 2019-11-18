import Foundation
import MetalKit
import ShaderTypes

//device char * buf [[ buffer(BufferIndexDebug) ]],
//constant uint * len [[ buffer(BufferIndexDebugLength) ]],
//                      uint gid [[ thread_position_in_grid ]],
//                      uint size [[ threads_per_grid ]])
//{
//Debug debug = Debug(buf, *len, gid, size);

class KernelDebugger {
    let stringBuffer: MTLBuffer
    var count: Int? = nil
    let label: String

    init(device: MTLDevice, length: Int = 1024 * 1024, label: String = "") {
        self.stringBuffer = device.makeBuffer(length: MemoryLayout<CChar>.stride * length, options: [.storageModeShared])!
        self.label = label
    }

    func encode(commandEncoder: MTLComputeCommandEncoder) {
        commandEncoder.setBuffer(stringBuffer, offset: 0, index: BufferIndex.debug.rawValue)
        var length = UInt(stringBuffer.length)
        commandEncoder.setBytes(&length, length: MemoryLayout<UInt>.stride, index: BufferIndex.debugLength.rawValue)
    }

    func wrap(_ commandBuffer: MTLCommandBuffer) -> MTLCommandBuffer {
        return KernelDebuggerCommandBufferProxy(commandBuffer, debugger: self)
    }

    var strings: [String] {
        guard let count = count else { return [] }

        var pointer = stringBuffer.contents().bindMemory(to: CChar.self, capacity: stringBuffer.length)
        var result: [String] = []
        for _ in 0..<count {
            result.append(String(cString: pointer))
            pointer = pointer.advanced(by: stringBuffer.length / count)
        }

        return result
    }

    func print() {
        guard let count = count else { return }

        for i in 0..<count {
            print(i)
        }
    }

    func print(_ i: Int) {
        let string = strings[i]
        let lines = string.split { $0.isNewline }
        for line in lines {
            Swift.print("\(label)[\(i)]: ", line)
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
