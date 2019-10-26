import Foundation
import MetalKit
import MetalPerformanceShaders

public final class MetalSimulator {
    private let device: MTLDevice

    private let mem: MemoryLayoutManager
    private let updateCompositeBodies: UpdateCompositeBodies
//    private let updateJoints: UpdateJoints
//    private let updateRigidBodies: UpdateRigidBodies
//    private let resetForces: ResetForces
//    private let applyPhysicsFields: ApplyPhysicsFields

    // FIXME shouldn't be public

//    public let compositeBodiesBuffer, jointsBuffer, rigidBodiesBuffer: MTLBuffer
//    public var rigidBodies: [RigidBody]

    private var fields: [PhysicsFieldStructConvertible] = []

    public func add(field: PhysicsFieldStructConvertible) {
        fields.append(field)
    }

    public init(device: MTLDevice, root: ArticulatedRigidBody) {
        self.device = device

        // Initialize buffers:
        self.mem = MemoryLayoutManager(device: device, root: root)
//        self.jointsBuffer = UpdateJoints.buffer(count: mem.rigidBody.count, device: device)

        // Initialize encoders:
//        self.resetForces = ResetForces(device: device, rigidBodiesBuffer: rigidBodiesBuffer, numRigidBodies: rigidBodies.count)
//        self.applyPhysicsFields = ApplyPhysicsFields(device: device, rigidBodiesBuffer: rigidBodiesBuffer, numRigidBodies: rigidBodies.count)
        self.updateCompositeBodies = UpdateCompositeBodies(device: device, memoryLayoutManager: mem)
//        self.updateJoints = UpdateJoints(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, numJoints: rigidBodies.count)
//        self.updateRigidBodies = UpdateRigidBodies(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, ranges: ranges)
    }

    public func encode(commandBuffer: MTLCommandBuffer, at time: TimeInterval) {
//        if let field = fields.first {
//            applyPhysicsFields.encode(commandBuffer: commandBuffer, field: field)
//        }
        updateCompositeBodies.encode(commandBuffer: commandBuffer)
//        updateJoints.encode(commandBuffer: commandBuffer, at: 1.0 / 60)
//        updateRigidBodies.encode(commandBuffer: commandBuffer)
//        resetForces.encode(commandBuffer: commandBuffer)
    }
}
