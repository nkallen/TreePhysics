import Foundation
import MetalKit

let maxBuffersInFlight = 3

class MetalKernel {
    let device: MTLDevice
    let computePipelineState: MTLComputePipelineState

    init(device: MTLDevice, name: String) {
        self.device = device
        let library = device.makeDefaultLibrary()!
        let kernelFunction = library.makeFunction(name: name)!
        self.computePipelineState = try! device.makeComputePipelineState(function: kernelFunction)
    }
}
