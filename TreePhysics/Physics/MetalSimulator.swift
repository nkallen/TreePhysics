import Foundation
import MetalKit

class MetalSimulator {
    private let device: MTLDevice, commandQueue: MTLCommandQueue

    private let updateCompositeBodies: UpdateCompositeBodiesKernel
    private let updateJoints: UpdateJointsKernel
    private let updateRigidBodies: UpdateRigidBodiesKernel
    private let resetForces: ResetForcesKernel
    private let applyPhysicsFields: ApplyPhysicsFieldsKernel

    private let compositeBodiesBuffer, jointsBuffer, rigidBodiesBuffer: MTLBuffer

    private var rigidBodies: [RigidBody]

    private var fields: [PhysicsField] = []

    func add(field: PhysicsField) {
        fields.append(field)
    }

    init(device: MTLDevice, root: RigidBody) {
        self.device = device
        self.commandQueue = device.makeCommandQueue()!

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

    func update(at time: TimeInterval, cb: @escaping (MTLBuffer, MTLBuffer, MTLBuffer, [RigidBody]) -> ()) {
        let commandBuffer = commandQueue.makeCommandBuffer()!

        resetForces.encode(commandBuffer: commandBuffer)
        applyPhysicsFields.encode(commandBuffer: commandBuffer, field: self.fields.first!)
        updateCompositeBodies.encode(commandBuffer: commandBuffer)
        updateJoints.encode(commandBuffer: commandBuffer, at: 1.0 / 60)
        updateRigidBodies.encode(commandBuffer: commandBuffer)

        commandBuffer.addCompletedHandler { _ in
            cb(self.compositeBodiesBuffer, self.jointsBuffer, self.rigidBodiesBuffer, self.rigidBodies)
        }

        commandBuffer.commit()
    }
}
