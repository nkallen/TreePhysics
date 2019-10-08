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
    #endif

    @available(OSX 10.15, *)
    var kernelStartTime: CFTimeInterval {
        return underlying.kernelStartTime
    }

    @available(OSX 10.15, *)
    var kernelEndTime: CFTimeInterval {
        return underlying.kernelEndTime
    }

    @available(OSX 10.15, *)
    var gpuStartTime: CFTimeInterval {
        return underlying.gpuStartTime
    }

    @available(OSX 10.15, *)
    var gpuEndTime: CFTimeInterval {
        return underlying.gpuEndTime
    }

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

    @available(OSX 10.15, *)
    func sampleCounters(sampleBuffer: MTLCounterSampleBuffer, sampleIndex: Int, barrier: Bool) {
        underlying.sampleCounters(sampleBuffer: sampleBuffer, sampleIndex: sampleIndex, barrier: barrier)
    }

    #if os(iOS)
    func setImageblockWidth(_ width: Int, height: Int) {
        underlying.setImageblockWidth(width, height: height)
    }

    func executeCommands(in indirectCommandBuffer: MTLIndirectCommandBuffer, with executionRange: NSRange) {
        underlying.executeCommands(in: indirectCommandBuffer, with: executionRange)
    }

    func executeCommands(in indirectCommandbuffer: MTLIndirectCommandBuffer, indirectBuffer indirectRangeBuffer: MTLBuffer, indirectBufferOffset: Int) {
        underlying.executeCommands(in: indirectCommandbuffer, indirectBuffer: indirectRangeBuffer, indirectBufferOffset: indirectBufferOffset)
    }
    #endif
}

class MTLDeviceProxy: NSObject, MTLDevice {
    let underlying: MTLDevice

    init(_ underlying: MTLDevice) {
        self.underlying = underlying
    }

    var name: String { return underlying.name }

    var registryID: UInt64 { return underlying.registryID }

    var maxThreadsPerThreadgroup: MTLSize { return underlying.maxThreadsPerThreadgroup }

    var readWriteTextureSupport: MTLReadWriteTextureTier { return underlying.readWriteTextureSupport }

    var argumentBuffersSupport: MTLArgumentBuffersTier { return underlying.argumentBuffersSupport }

    var areRasterOrderGroupsSupported: Bool { return underlying.areRasterOrderGroupsSupported }

    var currentAllocatedSize: Int { return underlying.currentAllocatedSize }

    func makeCommandQueue() -> MTLCommandQueue? {
        return underlying.makeCommandQueue()
    }

    func makeCommandQueue(maxCommandBufferCount: Int) -> MTLCommandQueue? {
        return underlying.makeCommandQueue(maxCommandBufferCount: maxCommandBufferCount)
    }

    func heapTextureSizeAndAlign(descriptor desc: MTLTextureDescriptor) -> MTLSizeAndAlign {
        return underlying.heapTextureSizeAndAlign(descriptor: desc)
    }

    func heapBufferSizeAndAlign(length: Int, options: MTLResourceOptions = []) -> MTLSizeAndAlign {
        return underlying.heapBufferSizeAndAlign(length: length, options: options)
    }

    func makeHeap(descriptor: MTLHeapDescriptor) -> MTLHeap? {
        return underlying.makeHeap(descriptor: descriptor)
    }

    func makeBuffer(length: Int, options: MTLResourceOptions = []) -> MTLBuffer? {
        return underlying.makeBuffer(length: length, options: options)
    }

    func makeBuffer(bytes pointer: UnsafeRawPointer, length: Int, options: MTLResourceOptions = []) -> MTLBuffer? {
        return underlying.makeBuffer(bytes: pointer, length: length, options: options)
    }

    func makeBuffer(bytesNoCopy pointer: UnsafeMutableRawPointer, length: Int, options: MTLResourceOptions = [], deallocator: ((UnsafeMutableRawPointer, Int) -> Void)? = nil) -> MTLBuffer? {
        return underlying.makeBuffer(bytesNoCopy: pointer, length: length, options: options, deallocator: deallocator)
    }

    func makeDepthStencilState(descriptor: MTLDepthStencilDescriptor) -> MTLDepthStencilState? {
        return underlying.makeDepthStencilState(descriptor: descriptor)
    }

    func makeTexture(descriptor: MTLTextureDescriptor) -> MTLTexture? {
        return underlying.makeTexture(descriptor: descriptor)
    }

    func makeSamplerState(descriptor: MTLSamplerDescriptor) -> MTLSamplerState? {
        return underlying.makeSamplerState(descriptor: descriptor)
    }

    func makeDefaultLibrary() -> MTLLibrary? {
        return underlying.makeDefaultLibrary()
    }

    func makeDefaultLibrary(bundle: Bundle) throws -> MTLLibrary {
        return try underlying.makeDefaultLibrary(bundle: bundle)
    }

    func makeLibrary(filepath: String) throws -> MTLLibrary {
        return try underlying.makeLibrary(filepath: filepath)
    }

    func makeLibrary(URL url: URL) throws -> MTLLibrary {
        return try underlying.makeLibrary(URL: url)
    }

    func makeLibrary(data: __DispatchData) throws -> MTLLibrary {
        return try underlying.makeLibrary(data: data)
    }

    func makeLibrary(source: String, options: MTLCompileOptions?) throws -> MTLLibrary {
        return try underlying.makeLibrary(source: source, options: options)
    }

    func makeLibrary(source: String, options: MTLCompileOptions?, completionHandler: @escaping MTLNewLibraryCompletionHandler) {
        return underlying.makeLibrary(source: source, options: options, completionHandler: completionHandler)
    }

    func makeRenderPipelineState(descriptor: MTLRenderPipelineDescriptor) throws -> MTLRenderPipelineState {
        return try underlying.makeRenderPipelineState(descriptor: descriptor)
    }

    func makeRenderPipelineState(descriptor: MTLRenderPipelineDescriptor, options: MTLPipelineOption, reflection: AutoreleasingUnsafeMutablePointer<MTLAutoreleasedRenderPipelineReflection?>?) throws -> MTLRenderPipelineState {
        return try underlying.makeRenderPipelineState(descriptor: descriptor, options: options, reflection: reflection)
    }

    func makeRenderPipelineState(descriptor: MTLRenderPipelineDescriptor, completionHandler: @escaping MTLNewRenderPipelineStateCompletionHandler) {
        return underlying.makeRenderPipelineState(descriptor: descriptor, completionHandler: completionHandler)
    }

    func makeRenderPipelineState(descriptor: MTLRenderPipelineDescriptor, options: MTLPipelineOption, completionHandler: @escaping MTLNewRenderPipelineStateWithReflectionCompletionHandler) {
        return underlying.makeRenderPipelineState(descriptor: descriptor, options: options, completionHandler: completionHandler)
    }

    func makeComputePipelineState(function computeFunction: MTLFunction) throws -> MTLComputePipelineState {
        return try underlying.makeComputePipelineState(function: computeFunction)
    }

    func makeComputePipelineState(function computeFunction: MTLFunction, options: MTLPipelineOption, reflection: AutoreleasingUnsafeMutablePointer<MTLAutoreleasedComputePipelineReflection?>?) throws -> MTLComputePipelineState {
        return try underlying.makeComputePipelineState(function: computeFunction, options: options, reflection: reflection)
    }

    func makeComputePipelineState(function computeFunction: MTLFunction, completionHandler: @escaping MTLNewComputePipelineStateCompletionHandler) {
        return underlying.makeComputePipelineState(function: computeFunction, completionHandler: completionHandler)
    }

    func makeComputePipelineState(function computeFunction: MTLFunction, options: MTLPipelineOption, completionHandler: @escaping MTLNewComputePipelineStateWithReflectionCompletionHandler) {
        return underlying.makeComputePipelineState(function: computeFunction, options: options, completionHandler: completionHandler)
    }

    func makeComputePipelineState(descriptor: MTLComputePipelineDescriptor, options: MTLPipelineOption, reflection: AutoreleasingUnsafeMutablePointer<MTLAutoreleasedComputePipelineReflection?>?) throws -> MTLComputePipelineState {
        return try underlying.makeComputePipelineState(descriptor: descriptor, options: options, reflection: reflection)
    }

    func makeComputePipelineState(descriptor: MTLComputePipelineDescriptor, options: MTLPipelineOption, completionHandler: @escaping MTLNewComputePipelineStateWithReflectionCompletionHandler) {
        return underlying.makeComputePipelineState(descriptor: descriptor, options: options, completionHandler: completionHandler)
    }

    func makeFence() -> MTLFence? {
        return underlying.makeFence()
    }

    func supportsFeatureSet(_ featureSet: MTLFeatureSet) -> Bool {
        return underlying.supportsFeatureSet(featureSet)
    }

    func supportsTextureSampleCount(_ sampleCount: Int) -> Bool {
        return underlying.supportsTextureSampleCount(sampleCount)
    }

    func minimumLinearTextureAlignment(for format: MTLPixelFormat) -> Int {
        return underlying.minimumLinearTextureAlignment(for: format)
    }

    func minimumTextureBufferAlignment(for format: MTLPixelFormat) -> Int {
        return underlying.minimumTextureBufferAlignment(for: format)
    }

    var maxThreadgroupMemoryLength: Int { return underlying.maxThreadgroupMemoryLength }

    var maxArgumentBufferSamplerCount: Int { return underlying.maxArgumentBufferSamplerCount }

    var areProgrammableSamplePositionsSupported: Bool { return underlying.areRasterOrderGroupsSupported }

    func __getDefaultSamplePositions(_ positions: UnsafeMutablePointer<MTLSamplePosition>, count: Int) {
        return underlying.__getDefaultSamplePositions(positions, count: count)
    }

    func makeArgumentEncoder(arguments: [MTLArgumentDescriptor]) -> MTLArgumentEncoder? {
        return underlying.makeArgumentEncoder(arguments: arguments)
    }

    func makeIndirectCommandBuffer(descriptor: MTLIndirectCommandBufferDescriptor, maxCommandCount maxCount: Int, options: MTLResourceOptions = []) -> MTLIndirectCommandBuffer? {
        return underlying.makeIndirectCommandBuffer(descriptor: descriptor, maxCommandCount: maxCount, options: options)
    }

    func makeEvent() -> MTLEvent? {
        return underlying.makeEvent()
    }

    func makeSharedEvent() -> MTLSharedEvent? {
        return underlying.makeSharedEvent()
    }

    func makeSharedEvent(handle sharedEventHandle: MTLSharedEventHandle) -> MTLSharedEvent? {
        return underlying.makeSharedEvent(handle: sharedEventHandle)
    }

    var maxBufferLength: Int { return underlying.maxBufferLength }

    func makeTexture(descriptor: MTLTextureDescriptor, iosurface: IOSurfaceRef, plane: Int) -> MTLTexture? {
        return underlying.makeTexture(descriptor: descriptor, iosurface: iosurface, plane: plane)
    }

    func makeSharedTexture(descriptor: MTLTextureDescriptor) -> MTLTexture? {
        return underlying.makeSharedTexture(descriptor: descriptor)
    }

    func makeSharedTexture(handle sharedHandle: MTLSharedTextureHandle) -> MTLTexture? {
        return underlying.makeSharedTexture(handle: sharedHandle)
    }

    @available(OSX 10.15, *)
    var hasUnifiedMemory: Bool { return underlying.hasUnifiedMemory }

    @available(OSX 10.15, *)
    var location: MTLDeviceLocation { return underlying.location}

    @available(OSX 10.15, *)
    var locationNumber: Int { return underlying.locationNumber }

    @available(OSX 10.15, *)
    var maxTransferRate: UInt64 { return underlying.maxTransferRate }

    @available(OSX 10.15, *)
    var areBarycentricCoordsSupported: Bool { return underlying.areBarycentricCoordsSupported }

    @available(OSX 10.15, *)
    var supportsShaderBarycentricCoordinates: Bool { return underlying.supportsShaderBarycentricCoordinates }

    @available(OSX 10.15, *)
    func supportsFamily(_ gpuFamily: MTLGPUFamily) -> Bool {
        return underlying.supportsFamily(gpuFamily)
    }

    @available(OSX 10.15, *)
    var peerGroupID: UInt64 { return underlying.peerGroupID }

    @available(OSX 10.15, *)
    var peerIndex: UInt32 { return underlying.peerIndex }

    @available(OSX 10.15, *)
    var peerCount: UInt32 { return underlying.peerCount }

    @available(OSX 10.15, *)
    var counterSets: [MTLCounterSet]? { return underlying.counterSets }

    @available(OSX 10.15, *)
    func makeCounterSampleBuffer(descriptor: MTLCounterSampleBufferDescriptor) throws -> MTLCounterSampleBuffer {
        return try underlying.makeCounterSampleBuffer(descriptor: descriptor)
    }

    @available(OSX 10.15, *)
    func __sampleTimestamps(_ cpuTimestamp: UnsafeMutablePointer<Int>, gpuTimestamp: UnsafeMutablePointer<Int>) {
        underlying.__sampleTimestamps(cpuTimestamp, gpuTimestamp: gpuTimestamp)
    }

    #if os(iOS)
    func supportsFamily(_ gpuFamily: MTLGPUFamily) -> Bool {
        return underlying.supportsFamily(gpuFamily)
    }

    func makeRenderPipelineState(tileDescriptor descriptor: MTLTileRenderPipelineDescriptor, options: MTLPipelineOption, reflection: AutoreleasingUnsafeMutablePointer<MTLAutoreleasedRenderPipelineReflection?>?) throws -> MTLRenderPipelineState {
        return try underlying.makeRenderPipelineState(tileDescriptor: descriptor, options: options, reflection: reflection)
    }

    func makeRenderPipelineState(tileDescriptor descriptor: MTLTileRenderPipelineDescriptor, options: MTLPipelineOption, completionHandler: @escaping MTLNewRenderPipelineStateWithReflectionCompletionHandler) {
        return underlying.makeRenderPipelineState(tileDescriptor: descriptor, options: options, completionHandler: completionHandler)
    }
    #else
    var isLowPower: Bool { return underlying.isLowPower }

    var isHeadless: Bool { return underlying.isHeadless }

    var isRemovable: Bool { return underlying.isRemovable }

    var recommendedMaxWorkingSetSize: UInt64 { return underlying.recommendedMaxWorkingSetSize }

    var isDepth24Stencil8PixelFormatSupported: Bool { return underlying.isDepth24Stencil8PixelFormatSupported }
    #endif
}
