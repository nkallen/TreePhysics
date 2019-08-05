import Foundation
import MetalKit

// The 256 byte aligned size of our uniform structure
let alignedUniformsSize = (MemoryLayout<float3x3>.size & ~0xFF) + 0x100
let maxBuffersInFlight = 3

class Foo {
    let device: MTLDevice
    let computePipelineState: MTLComputePipelineState
    let inFlightSemaphore = DispatchSemaphore(value: maxBuffersInFlight)
    let buffer: MTLBuffer
    let outBuffer: MTLBuffer
    let commandQueue: MTLCommandQueue
    var bufferOffset: Int

    init() {
        self.device = MTLCreateSystemDefaultDevice()!
        let library = device.makeDefaultLibrary()!
        let kernelFunction = library.makeFunction(name: "diagonalize")!
        self.computePipelineState = try! device.makeComputePipelineState(function: kernelFunction)

        let bufferSize = MemoryLayout<float3>.size * 1024
        self.buffer = device.makeBuffer(length: bufferSize, options: [.storageModeShared])!
        self.outBuffer = device.makeBuffer(length: bufferSize, options: [.storageModeShared])!
        self.bufferOffset = 0
        self.commandQueue = device.makeCommandQueue()!
    }

    func run(_ matrix: float3x3, cb: @escaping (MTLBuffer) -> ()) {
        let foo = UnsafeMutableRawPointer(buffer.contents() + bufferOffset).bindMemory(to: float3x3.self, capacity: 1)
        foo[0] = matrix

        //        let _ = inFlightSemaphore.wait(timeout: DispatchTime.distantFuture)

        let commandBuffer = commandQueue.makeCommandBuffer()!
        commandBuffer.addCompletedHandler{ [weak self] commandBuffer in
            if let strongSelf = self {
//                strongSelf.inFlightSemaphore.signal()

                cb(strongSelf.outBuffer)
            }
        }


        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.setComputePipelineState(computePipelineState)
        commandEncoder.label  = "Test"
        commandEncoder.setBuffer(buffer, offset: bufferOffset, index: 0)
        commandEncoder.setBuffer(outBuffer, offset: 0, index: 1)

        let width = computePipelineState.threadExecutionWidth
        let height = computePipelineState.maxTotalThreadsPerThreadgroup / width
        let threadsPerThreadgroup = MTLSizeMake(width, height, 1)
        let threadsPerGrid = MTLSize(
            width: 1,
            height: 1024,
            depth: 1)
        commandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        commandEncoder.endEncoding()

        commandBuffer.commit()
    }
}
