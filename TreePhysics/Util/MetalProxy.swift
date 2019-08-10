import Foundation
import MetalKit

class MTLCommandBufferProxy: NSObject, MTLCommandBuffer {
    let underlying: MTLCommandBuffer

    init(_ underlying: MTLCommandBuffer) {
        self.underlying = underlying
    }

    var device: MTLDevice { return underlying.device }

    var commandQueue: MTLCommandQueue { return underlying.commandQueue }

    var retainedReferences: Bool { return underlying.retainedReferences }

    var label: String? {
        get {
            return underlying.label
        }
        set {
            underlying.label = newValue
        }
    }

    func enqueue() {
        underlying.enqueue()
    }

    func commit() {
        underlying.commit()
    }

    func addScheduledHandler(_ block: @escaping MTLCommandBufferHandler) {
        underlying.addScheduledHandler(block)
    }

    func present(_ drawable: MTLDrawable) {
        underlying.present(drawable)
    }

    func present(_ drawable: MTLDrawable, atTime presentationTime: CFTimeInterval) {
        underlying.present(drawable, atTime: presentationTime)
    }

    #if os(iOS)
    func present(_ drawable: MTLDrawable, afterMinimumDuration presentationTime: CFTimeInterval) {
        underlying.present(drawable, afterMinimumDuration: presentationTime)
    }

    var kernelStartTime: CFTimeInterval {
        return underlying.kernelStartTime
    }

    var kernelEndTime: CFTimeInterval {
        return underlying.kernelEndTime
    }

    var gpuStartTime: CFTimeInterval {
        return underlying.gpuStartTime
    }

    var gpuEndTime: CFTimeInterval {
        return underlying.gpuEndTime
    }
    #endif

    func waitUntilScheduled() {
        underlying.waitUntilScheduled()
    }

    func addCompletedHandler(_ block: @escaping MTLCommandBufferHandler) {
        underlying.addCompletedHandler(block)
    }

    func waitUntilCompleted() {
        underlying.waitUntilCompleted()
    }

    var status: MTLCommandBufferStatus { return underlying.status }

    var error: Error? { return underlying.error }

    func makeBlitCommandEncoder() -> MTLBlitCommandEncoder? {
        return underlying.makeBlitCommandEncoder()
    }

    func makeRenderCommandEncoder(descriptor renderPassDescriptor: MTLRenderPassDescriptor) -> MTLRenderCommandEncoder? {
        return underlying.makeRenderCommandEncoder(descriptor: renderPassDescriptor)
    }

    func makeComputeCommandEncoder() -> MTLComputeCommandEncoder? {
        return underlying.makeComputeCommandEncoder()
    }

    func makeComputeCommandEncoder(dispatchType: MTLDispatchType) -> MTLComputeCommandEncoder? {
        return underlying.makeComputeCommandEncoder(dispatchType: dispatchType)
    }

    func encodeWaitForEvent(_ event: MTLEvent, value: UInt64) {
        underlying.encodeWaitForEvent(event, value: value)
    }

    func encodeSignalEvent(_ event: MTLEvent, value: UInt64) {
        underlying.encodeSignalEvent(event, value: value)
    }

    func makeParallelRenderCommandEncoder(descriptor renderPassDescriptor: MTLRenderPassDescriptor) -> MTLParallelRenderCommandEncoder? {
        return underlying.makeParallelRenderCommandEncoder(descriptor: renderPassDescriptor)
    }

    func pushDebugGroup(_ string: String) {
        underlying.pushDebugGroup(string)
    }

    func popDebugGroup() {
        underlying.popDebugGroup()
    }
}

class MTLComputeCommandEncoderProxy: NSObject, MTLComputeCommandEncoder {
    let underlying: MTLComputeCommandEncoder

    init(_ underlying: MTLComputeCommandEncoder) {
        self.underlying = underlying
    }

    var dispatchType: MTLDispatchType { return underlying.dispatchType }

    func setComputePipelineState(_ state: MTLComputePipelineState) {
        underlying.setComputePipelineState(state)
    }

    func setBytes(_ bytes: UnsafeRawPointer, length: Int, index: Int) {
        underlying.setBytes(bytes, length: length, index: index)
    }

    func setBuffer(_ buffer: MTLBuffer?, offset: Int, index: Int) {
        underlying.setBuffer(buffer, offset: offset, index: index)
    }

    func setBufferOffset(_ offset: Int, index: Int) {
        underlying.setBufferOffset(offset, index: index)
    }

    func __setBuffers(_ buffers: UnsafePointer<MTLBuffer?>, offsets: UnsafePointer<Int>, with range: NSRange) {
        underlying.__setBuffers(buffers, offsets: offsets, with: range)
    }

    func setTexture(_ texture: MTLTexture?, index: Int) {
        underlying.setTexture(texture, index: index)
    }

    func __setTextures(_ textures: UnsafePointer<MTLTexture?>, with range: NSRange) {
        underlying.__setTextures(textures, with: range)
    }

    func setSamplerState(_ sampler: MTLSamplerState?, index: Int) {
        underlying.setSamplerState(sampler, index: index)
    }

    func __setSamplerStates(_ samplers: UnsafePointer<MTLSamplerState?>, with range: NSRange) {
        underlying.__setSamplerStates(samplers, with: range)
    }

    func setSamplerState(_ sampler: MTLSamplerState?, lodMinClamp: Float, lodMaxClamp: Float, index: Int) {
        underlying.setSamplerState(sampler, lodMinClamp: lodMinClamp, lodMaxClamp: lodMaxClamp, index: index)
    }

    func __setSamplerStates(_ samplers: UnsafePointer<MTLSamplerState?>, lodMinClamps: UnsafePointer<Float>, lodMaxClamps: UnsafePointer<Float>, with range: NSRange) {
        underlying.__setSamplerStates(samplers, lodMinClamps: lodMinClamps, lodMaxClamps: lodMaxClamps, with: range)
    }

    func setThreadgroupMemoryLength(_ length: Int, index: Int) {
        underlying.setThreadgroupMemoryLength(length, index: index)
    }

    func setStageInRegion(_ region: MTLRegion) {
        underlying.setStageInRegion(region)
    }

    func setStageInRegionWithIndirectBuffer(_ indirectBuffer: MTLBuffer, indirectBufferOffset: Int) {
        underlying.setStageInRegionWithIndirectBuffer(indirectBuffer, indirectBufferOffset: indirectBufferOffset)
    }

    func dispatchThreadgroups(_ threadgroupsPerGrid: MTLSize, threadsPerThreadgroup: MTLSize) {
        underlying.dispatchThreadgroups(threadgroupsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
    }

    func dispatchThreadgroups(indirectBuffer: MTLBuffer, indirectBufferOffset: Int, threadsPerThreadgroup: MTLSize) {
        underlying.dispatchThreadgroups(indirectBuffer: indirectBuffer, indirectBufferOffset: indirectBufferOffset, threadsPerThreadgroup: threadsPerThreadgroup)
    }

    func dispatchThreads(_ threadsPerGrid: MTLSize, threadsPerThreadgroup: MTLSize) {
        underlying.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
    }

    func updateFence(_ fence: MTLFence) {
        underlying.updateFence(fence)
    }

    func waitForFence(_ fence: MTLFence) {
        underlying.waitForFence(fence)
    }

    func useResource(_ resource: MTLResource, usage: MTLResourceUsage) {
        underlying.useResource(resource, usage: usage)
    }

    func __use(_ resources: UnsafePointer<MTLResource>, count: Int, usage: MTLResourceUsage) {
        underlying.__use(resources, count: count, usage: usage)
    }

    func useHeap(_ heap: MTLHeap) {
        underlying.useHeap(heap)
    }

    func __use(_ heaps: UnsafePointer<MTLHeap>, count: Int) {
        underlying.__use(heaps, count: count)
    }

    func memoryBarrier(scope: MTLBarrierScope) {
        underlying.memoryBarrier(scope: scope)
    }

    func __memoryBarrier(resources: UnsafePointer<MTLResource>, count: Int) {
        underlying.__memoryBarrier(resources: resources, count: count)
    }

    var device: MTLDevice {
        return underlying.device
    }

    var label: String? {
        get {
            return underlying.label
        }
        set {
            underlying.label = newValue
        }
    }

    func endEncoding() {
        underlying.endEncoding()
    }

    func insertDebugSignpost(_ string: String) {
        underlying.insertDebugSignpost(string)
    }

    func pushDebugGroup(_ string: String) {
        underlying.pushDebugGroup(string)
    }

    func popDebugGroup() {
        underlying.popDebugGroup()
    }

    #if os(iOS)
    func setImageblockWidth(_ width: Int, height: Int) {
        underlying.setImageblockWidth(width, height: height)
    }
    #endif
}
