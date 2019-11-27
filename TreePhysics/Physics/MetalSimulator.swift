import Foundation
import MetalKit
import MetalPerformanceShaders

public final class MetalSimulator {
    private let device: MTLDevice
    let debug: KernelDebugger? = nil

    private let mem: MemoryLayoutManager
    private let applyPhysicsFields: ApplyPhysicsFields
    private let updateCompositeBodies: UpdateCompositeBodies
    private let updateJoints: UpdateJoints
    private let updateRigidBodies: UpdateRigidBodies
    private let freeBodies: FreeBodies
    private let updateFreeBodies: UpdateFreeBodies

    public init(device: MTLDevice, mem: MemoryLayoutManager) {
        self.device = device

//        self.debug = KernelDebugger(device: device, length: mem.rigidBodies.ranges.last!.count * 4096)

        self.mem = mem
        self.applyPhysicsFields    = ApplyPhysicsFields(device: device, memoryLayoutManager: mem)
        self.updateCompositeBodies = UpdateCompositeBodies(device: device, memoryLayoutManager: mem)
        self.updateJoints          = UpdateJoints(device: device, memoryLayoutManager: mem)
        self.updateRigidBodies     = UpdateRigidBodies(device: device, memoryLayoutManager: mem)
        self.freeBodies            = FreeBodies(device: device, memoryLayoutManager: mem)
        self.updateFreeBodies      = UpdateFreeBodies(device: device, memoryLayoutManager: mem)
    }

    public func encode(commandBuffer: MTLCommandBuffer, at time: TimeInterval) {
        let time = Float(time)

        commandBuffer.pushDebugGroup("MetalSimulator")
        applyPhysicsFields.encode(commandBuffer: commandBuffer, at: time)
        updateCompositeBodies.encode(commandBuffer: commandBuffer)
        updateJoints.encode(commandBuffer: commandBuffer, at: time)
        updateRigidBodies.encode(commandBuffer: commandBuffer)
        updateFreeBodies.encode(commandBuffer: commandBuffer, at: time)
        freeBodies.encode(commandBuffer: commandBuffer)
        commandBuffer.popDebugGroup()
    }
}
