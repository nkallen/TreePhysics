import Foundation
import MetalKit
import MetalPerformanceShaders

public final class MetalSimulator {
    private let device: MTLDevice
    let debug: KernelDebugger? = nil

    private let mem: MemoryLayoutManager
    private let updateCompositeBodies: UpdateCompositeBodies
    private let freeBodies: FreeBodies
    private let updateJoints: UpdateJoints
    private let updateRigidBodies: UpdateRigidBodies
    private let applyPhysicsFields: ApplyPhysicsFields

    public init(device: MTLDevice, mem: MemoryLayoutManager) {
        self.device = device

//        self.debug = KernelDebugger(device: device, length: mem.rigidBodies.ranges.last!.count * 4096)

        self.mem = mem
        self.applyPhysicsFields = ApplyPhysicsFields(memoryLayoutManager: mem)
        self.updateCompositeBodies = UpdateCompositeBodies(memoryLayoutManager: mem)
        self.freeBodies = FreeBodies(memoryLayoutManager: mem)
        self.updateJoints = UpdateJoints(memoryLayoutManager: mem)
        self.updateRigidBodies = UpdateRigidBodies(memoryLayoutManager: mem)
    }

    public func encode(commandBuffer: MTLCommandBuffer, at time: TimeInterval) {
        commandBuffer.pushDebugGroup("MetalSimulator")
        applyPhysicsFields.encode(commandBuffer: commandBuffer, at: Float(time))
        updateCompositeBodies.encode(commandBuffer: commandBuffer)
        freeBodies.encode(commandBuffer: commandBuffer)
        updateJoints.encode(commandBuffer: commandBuffer, at: Float(time))
        updateRigidBodies.encode(commandBuffer: commandBuffer)
        commandBuffer.popDebugGroup()
    }
}
