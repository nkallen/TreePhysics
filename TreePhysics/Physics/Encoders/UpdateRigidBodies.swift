import Foundation
import MetalKit
import Metal
import ShaderTypes

public final class UpdateRigidBodies: MetalKernelEncoder {
    private let argumentEncoder: ArgumentEncoder

    init(device: MTLDevice = MTLCreateSystemDefaultDevice()!, memoryLayoutManager: MemoryLayoutManager) {
        let library = device.makeDefaultLibrary()!
        let constantValues = MTLFunctionConstantValues()
        var rangeCount = memoryLayoutManager.rigidBodies.ranges.count
        constantValues.setConstantValue(&rangeCount, type: .int, index: FunctionConstantIndex.rangeCount.rawValue)
        let function = try! library.makeFunction(name: "updateRigidBodies", constantValues: constantValues)

        self.argumentEncoder = ArgumentEncoder(memoryLayoutManager: memoryLayoutManager)

        super.init(device: device, function: function)
    }

    public func encode(commandBuffer: MTLCommandBuffer) {
        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.setComputePipelineState(computePipelineState)
        commandEncoder.label  = "Update Rigid Bodies"
        argumentEncoder.encode(commandEncoder: commandEncoder)

        let threadGroupWidth = computePipelineState.maxTotalThreadsPerThreadgroup
        let threadsPerThreadgroup = MTLSizeMake(threadGroupWidth, 1, 1)
        let threadsPerGrid = MTLSize(
            width: argumentEncoder.mem.rigidBodies.maxRangeWidth,
            height: 1,
            depth: 1)

        commandEncoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        commandEncoder.endEncoding()
    }
}

extension UpdateRigidBodies {
    class ArgumentEncoder {
        let mem: MemoryLayoutManager

        init(memoryLayoutManager: MemoryLayoutManager) {
            self.mem = memoryLayoutManager
        }

        func encode(commandEncoder: MTLComputeCommandEncoder) {
            let bufs = [
                mem.rigidBodies.parentIdBuffer,
                mem.rigidBodies.localPivotBuffer,
                mem.rigidBodies.localInertiaTensorBuffer,
                mem.rigidBodies.localJointPositionBuffer,
                mem.rigidBodies.localJointOrientationBuffer,
                mem.joints.thetaBuffer,

                mem.rigidBodies.pivotBuffer,
                mem.rigidBodies.centerOfMassBuffer,
                mem.rigidBodies.inertiaTensorBuffer,
                mem.rigidBodies.orientationBuffer,
                mem.rigidBodies.jointOrientationBuffer,
                mem.rigidBodies.velocityBuffer,
                mem.rigidBodies.angularVelocityBuffer,
                mem.rigidBodies.accelerationBuffer,
                mem.rigidBodies.angularAccelerationBuffer,
            ]
            commandEncoder.setBuffers(bufs, offsets: [Int](repeating: 0, count: bufs.count), range: 0..<bufs.count)

            let ranges = mem.rigidBodies.ranges

            var deltas = ranges.map { range in
                UInt16(range.upperBound - range.lowerBound)
            }
            commandEncoder.setBytes(&deltas, length: MemoryLayout<UInt16>.stride * deltas.count, index: bufs.count)
        }
    }

}
