import Foundation
import MetalKit
import MetalPerformanceShaders

class MetalSimulator {
    private let device: MTLDevice

    private let updateCompositeBodies: UpdateCompositeBodiesKernel
    private let updateJoints: UpdateJointsKernel
    private let updateRigidBodies: UpdateRigidBodiesKernel
    private let resetForces: ResetForcesKernel
    private let applyPhysicsFields: ApplyPhysicsFieldsKernel

    internal let compositeBodiesBuffer, jointsBuffer, rigidBodiesBuffer: MTLBuffer

    internal var rigidBodies: [RigidBody]

    private var fields: [PhysicsField] = []

    func add(field: PhysicsField) {
        fields.append(field)
    }

    init(device: MTLDevice, root: RigidBody) {
        self.device = device

        // Initialize buffers:
        let (rigidBodies, rigidBodiesBuffer, ranges) = UpdateCompositeBodiesKernel.buffer(root: root, device: device)
        self.rigidBodies = rigidBodies
        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.compositeBodiesBuffer = device.makeBuffer(
            length: MemoryLayout<CompositeBodyStruct>.stride * rigidBodies.count,
            options: [.storageModeShared])!
        self.jointsBuffer = UpdateJointsKernel.buffer(count: rigidBodies.count, device: device)

        // Initialize encoders:
        self.resetForces = ResetForcesKernel(device: device, rigidBodiesBuffer: rigidBodiesBuffer, numRigidBodies: rigidBodies.count)
        self.applyPhysicsFields = ApplyPhysicsFieldsKernel(device: device, rigidBodiesBuffer: rigidBodiesBuffer, numRigidBodies: rigidBodies.count)
        self.updateCompositeBodies = UpdateCompositeBodiesKernel(device: device, rigidBodiesBuffer: rigidBodiesBuffer, ranges: ranges, compositeBodiesBuffer: compositeBodiesBuffer)
        self.updateJoints = UpdateJointsKernel(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, numJoints: rigidBodies.count)
        self.updateRigidBodies = UpdateRigidBodiesKernel(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, ranges: ranges)
    }

    func encode(commandBuffer: MTLCommandBuffer, at time: TimeInterval) {
        resetForces.encode(commandBuffer: commandBuffer)
        applyPhysicsFields.encode(commandBuffer: commandBuffer, field: self.fields.first!)
        updateCompositeBodies.encode(commandBuffer: commandBuffer)
        updateJoints.encode(commandBuffer: commandBuffer, at: 1.0 / 60)
        updateRigidBodies.encode(commandBuffer: commandBuffer)
    }
}
