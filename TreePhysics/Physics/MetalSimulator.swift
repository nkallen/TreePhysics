import Foundation
import MetalKit
import MetalPerformanceShaders

class MetalSimulator {
    private let device: MTLDevice

    private let updateCompositeBodies: UpdateCompositeBodies
    private let updateJoints: UpdateJoints
    private let updateRigidBodies: UpdateRigidBodies
    private let resetForces: ResetForces
    private let applyPhysicsFields: ApplyPhysicsFields

    internal let compositeBodiesBuffer, jointsBuffer, rigidBodiesBuffer: MTLBuffer

    internal var rigidBodies: [RigidBody]

    private var fields: [PhysicsField] = []

    func add(field: PhysicsField) {
        fields.append(field)
    }

    init(device: MTLDevice, root: RigidBody) {
        self.device = device

        // Initialize buffers:
        let (rigidBodies, rigidBodiesBuffer, ranges) = UpdateCompositeBodies.buffer(root: root, device: device)
        self.rigidBodies = rigidBodies
        self.rigidBodiesBuffer = rigidBodiesBuffer
        self.compositeBodiesBuffer = device.makeBuffer(
            length: MemoryLayout<CompositeBodyStruct>.stride * rigidBodies.count,
            options: [.storageModePrivate])!
        self.jointsBuffer = UpdateJoints.buffer(count: rigidBodies.count, device: device)

        // Initialize encoders:
        self.resetForces = ResetForces(device: device, rigidBodiesBuffer: rigidBodiesBuffer, numRigidBodies: rigidBodies.count)
        self.applyPhysicsFields = ApplyPhysicsFields(device: device, rigidBodiesBuffer: rigidBodiesBuffer, numRigidBodies: rigidBodies.count)
        self.updateCompositeBodies = UpdateCompositeBodies(device: device, rigidBodiesBuffer: rigidBodiesBuffer, ranges: ranges, compositeBodiesBuffer: compositeBodiesBuffer)
        self.updateJoints = UpdateJoints(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, numJoints: rigidBodies.count)
        self.updateRigidBodies = UpdateRigidBodies(device: device, rigidBodiesBuffer: rigidBodiesBuffer, compositeBodiesBuffer: compositeBodiesBuffer, jointsBuffer: jointsBuffer, ranges: ranges)
    }

    func encode(commandBuffer: MTLCommandBuffer, at time: TimeInterval) {
        resetForces.encode(commandBuffer: commandBuffer)
        applyPhysicsFields.encode(commandBuffer: commandBuffer, field: self.fields.first!)
        updateCompositeBodies.encode(commandBuffer: commandBuffer)
        updateJoints.encode(commandBuffer: commandBuffer, at: 1.0 / 60)
        updateRigidBodies.encode(commandBuffer: commandBuffer)
    }
}