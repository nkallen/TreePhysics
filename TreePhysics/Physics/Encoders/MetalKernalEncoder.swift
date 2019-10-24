import Foundation
import MetalKit
import ShaderTypes

let maxBuffersInFlight = 3

class MetalKernelEncoder {
    let device: MTLDevice
    let computePipelineState: MTLComputePipelineState

    init(device: MTLDevice, name: String) {
        self.device = device
        // FIXME cf https://stackoverflow.com/questions/46742403/metal-file-as-part-of-an-ios-framework for a better way to embed
        let library = device.makeDefaultLibrary()!
        let function = library.makeFunction(name: name)!
        self.computePipelineState = try! device.makeComputePipelineState(function: function)
    }

    init(device: MTLDevice, function: MTLFunction) {
        self.device = device
        self.computePipelineState = try! device.makeComputePipelineState(function: function)
    }
}
