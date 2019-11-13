import Foundation
import MetalKit

class SharedBuffersMTLDevice: MTLDeviceProxy {
    override func makeBuffer(length: Int, options: MTLResourceOptions = []) -> MTLBuffer? {
        return underlying.makeBuffer(length: length, options: changeStorageMode(of: options))
    }

    override func makeBuffer(bytes pointer: UnsafeRawPointer, length: Int, options: MTLResourceOptions = []) -> MTLBuffer? {
        return underlying.makeBuffer(bytes: pointer, length: length, options: changeStorageMode(of: options))
    }

    override func makeBuffer(bytesNoCopy pointer: UnsafeMutableRawPointer, length: Int, options: MTLResourceOptions = [], deallocator: ((UnsafeMutableRawPointer, Int) -> Void)? = nil) -> MTLBuffer? {
        return underlying.makeBuffer(bytesNoCopy: pointer, length: length, options: changeStorageMode(of: options), deallocator: deallocator)
    }

    private func changeStorageMode(of options: MTLResourceOptions) -> MTLResourceOptions {
        var options = options
        options.remove(.storageModePrivate)
        options.insert(.storageModeShared)
        return options
    }
}
