import Foundation
import MetalKit
import MetalPerformanceShaders

public final class MetalSimulator {
    private let device: MTLDevice
    let debug: KernelDebugger

    private let mem: MemoryLayoutManager
    private let updateCompositeBodies: UpdateCompositeBodies
    private let updateJoints: UpdateJoints
    private let updateRigidBodies: UpdateRigidBodies
//    private let resetForces: ResetForces
//    private let applyPhysicsFields: ApplyPhysicsFields

//    private var fields: [PhysicsFieldStructConvertible] = []
//
//    public func add(field: PhysicsFieldStructConvertible) {
//        fields.append(field)
//    }

    public init(device: MTLDevice, mem: MemoryLayoutManager) {
        self.device = device

        self.debug = KernelDebugger(device: device, length: mem.rigidBodies.ranges.last!.count * 4096)

        self.mem = mem
        self.updateCompositeBodies = UpdateCompositeBodies(memoryLayoutManager: mem)
        self.updateJoints = UpdateJoints(memoryLayoutManager: mem)
        self.updateRigidBodies = UpdateRigidBodies(memoryLayoutManager: mem)

//        self.jointsBuffer = UpdateJoints.buffer(count: mem.rigidBody.count, device: device)

        // Initialize encoders:
//        self.resetForces = ResetForces(device: device, rigidBodiesBuffer: rigidBodiesBuffer, numRigidBodies: rigidBodies.count)
//        self.applyPhysicsFields = ApplyPhysicsFields(device: device, rigidBodiesBuffer: rigidBodiesBuffer, numRigidBodies: rigidBodies.count)
//        self.updateJoints = UpdateJoints(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, numJoints: rigidBodies.count)
//        self.updateRigidBodies = UpdateRigidBodies(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, ranges: ranges)
    }

    public func encode(commandBuffer: MTLCommandBuffer, at time: TimeInterval) {
//        if let field = fields.first {
//            applyPhysicsFields.encode(commandBuffer: commandBuffer, field: field)
//        }
        updateCompositeBodies.encode(commandBuffer: debug.wrap(commandBuffer))
//        updateJoints.encode(commandBuffer: commandBuffer, at: Float(time))
//        updateRigidBodies.encode(commandBuffer: commandBuffer)
//        resetForces.encode(commandBuffer: commandBuffer)
    }
}
