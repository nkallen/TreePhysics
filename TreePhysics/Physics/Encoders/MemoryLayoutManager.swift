import Foundation
import simd
import MetalKit
import ShaderTypes

extension half {
    static let leastNormalMagnitude: half = Half(0x1.0p-14)
}

public final class MemoryLayoutManager {
    final class RigidBodies {
        let index: [RigidBody:Int]
        let ranges: [Range<Int>]
        let maxClimberCount: Int
        let maxChildCount: Int
        let count: Int
        let childCountBuffer, childIndexBuffer, parentIdBuffer, climberCountBuffer, massBuffer, pivotBuffer, localPivotBuffer, forceBuffer, torqueBuffer, centerOfMassBuffer, orientationBuffer, inertiaTensorBuffer, localInertiaTensorBuffer, jointDampingBuffer, jointStiffnessBuffer, localJointPositionBuffer, localJointOrientationBuffer, jointOrientationBuffer, velocityBuffer, angularVelocityBuffer, accelerationBuffer, angularAccelerationBuffer, areaBuffer: MTLBuffer

        init(device: MTLDevice, root: ArticulatedRigidBody) {
            var ranges: [Range<Int>] = []
            let levels = root.levels()
            var offset = 0
            var id = 0
            var index = [RigidBody:Int]()

            // Step 1: Determine the buffer memory layout (i.e., the index and the ranges of work)
            var maxClimberCount = 0
            var maxChildCount = 0
            for level in levels {
                for unitOfWork in level {
                    index[unitOfWork.rigidBody] = id
                    id += 1
                    maxClimberCount = max(maxClimberCount, unitOfWork.climbers.count)
                    maxChildCount = max(maxChildCount, unitOfWork.rigidBody.childJoints.count)
                }

                let range = offset..<(offset+level.count)
                ranges.append(range)
                offset += level.count
            }
            self.ranges = ranges
            self.maxClimberCount = maxClimberCount
            self.maxChildCount = maxChildCount

            for level in levels {
                for unitOfWork in level {
                    for climber in unitOfWork.climbers {
                        index[climber] = id
                        id += 1
                    }
                }
            }
            self.index = index

            // Step 2: Allocate the shared buffers
            self.count = id

            let childCountBuffer = device.makeBuffer(length: count * MemoryLayout<UInt8>.stride, options: [.storageModeShared])!
            let childIndexBuffer = device.makeBuffer(length: count * MemoryLayout<UInt8>.stride, options: [.storageModeShared])!
            let parentIdBuffer = device.makeBuffer(length: count * MemoryLayout<UInt16>.stride, options: [.storageModeShared])!
            let climberCountBuffer = device.makeBuffer(length: count * MemoryLayout<UInt8>.stride, options: [.storageModeShared])!
            let massBuffer = device.makeBuffer(length: count * MemoryLayout<half>.stride, options: [.storageModeShared])!
            let pivotBuffer = device.makeBuffer(length: count * MemoryLayout<packed_half3>.stride, options: [.storageModeShared])!
            let localPivotBuffer = device.makeBuffer(length: count * MemoryLayout<packed_half3>.stride, options: [.storageModeShared])!
            let forceBuffer = device.makeBuffer(length: count * MemoryLayout<packed_half3>.stride, options: [.storageModeShared])!
            let torqueBuffer = device.makeBuffer(length: count * MemoryLayout<packed_half3>.stride, options: [.storageModeShared])!
            let centerOfMassBuffer = device.makeBuffer(length: count * MemoryLayout<packed_half3>.stride, options: [.storageModeShared])!
            let orientationBuffer = device.makeBuffer(length: count * MemoryLayout<simd_quath>.stride, options: [.storageModeShared])!
            let inertiaTensorBuffer = device.makeBuffer(length: count * MemoryLayout<InertiaTensor>.stride, options: [.storageModeShared])!
            let localInertiaTensorBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            let jointDampingBuffer = device.makeBuffer(length: count * MemoryLayout<half>.stride, options: [.storageModeShared])!
            let jointStiffnessBuffer = device.makeBuffer(length: count * MemoryLayout<half>.stride, options: [.storageModeShared])!
            let localJointPositionBuffer = device.makeBuffer(length: count * MemoryLayout<packed_half3>.stride, options: [.storageModeShared])!
            let localJointOrientationBuffer = device.makeBuffer(length: count * MemoryLayout<simd_quath>.stride, options: [.storageModeShared])!
            let jointOrientationBuffer = device.makeBuffer(length: count * MemoryLayout<simd_quath>.stride, options: [.storageModeShared])!
            let velocityBuffer = device.makeBuffer(length: count * MemoryLayout<packed_half3>.stride, options: [.storageModeShared])!
            let angularVelocityBuffer = device.makeBuffer(length: count * MemoryLayout<packed_half3>.stride, options: [.storageModeShared])!
            let accelerationBuffer = device.makeBuffer(length: count * MemoryLayout<packed_half3>.stride, options: [.storageModeShared])!
            let angularAccelerationBuffer = device.makeBuffer(length: count * MemoryLayout<packed_half3>.stride, options: [.storageModeShared])!
            let areaBuffer = device.makeBuffer(length: count * MemoryLayout<half>.stride, options: [.storageModeShared])!

            let childCount = childCountBuffer.contents().bindMemory(to: UInt8.self, capacity: count)
            let childIndex = childIndexBuffer.contents().bindMemory(to: UInt8.self, capacity: count)
            let parentId = parentIdBuffer.contents().bindMemory(to: UInt16.self, capacity: count)
            let climberCount = climberCountBuffer.contents().bindMemory(to: UInt8.self, capacity: count)
            let mass = massBuffer.contents().bindMemory(to: half.self, capacity: count)
            let pivot = pivotBuffer.contents().bindMemory(to: packed_half3.self, capacity: count)
            let localPivot = localPivotBuffer.contents().bindMemory(to: packed_half3.self, capacity: count)
            let force = forceBuffer.contents().bindMemory(to: packed_half3.self, capacity: count)
            let torque = torqueBuffer.contents().bindMemory(to: packed_half3.self, capacity: count)
            let centerOfMass = centerOfMassBuffer.contents().bindMemory(to: packed_half3.self, capacity: count)
            let orientation = orientationBuffer.contents().bindMemory(to: simd_quath.self, capacity: count)
            let inertiaTensor = inertiaTensorBuffer.contents().bindMemory(to: InertiaTensor.self, capacity: count)
            let localInertiaTensor = localInertiaTensorBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
            let jointDamping = jointDampingBuffer.contents().bindMemory(to: half.self, capacity: count)
            let jointStiffness = jointStiffnessBuffer.contents().bindMemory(to: half.self, capacity: count)
            let localJointPosition = localJointPositionBuffer.contents().bindMemory(to: packed_half3.self, capacity: count)
            let localJointOrientation = localJointOrientationBuffer.contents().bindMemory(to: simd_quath.self, capacity: count)
            let jointOrientation = jointOrientationBuffer.contents().bindMemory(to: simd_quath.self, capacity: count)
            let velocity = velocityBuffer.contents().bindMemory(to: packed_half3.self, capacity: count)
            let angularVelocity = angularVelocityBuffer.contents().bindMemory(to: packed_half3.self, capacity: count)
            let acceleration = accelerationBuffer.contents().bindMemory(to: packed_half3.self, capacity: count)
            let angularAcceleration = angularAccelerationBuffer.contents().bindMemory(to: packed_half3.self, capacity: count)
            let area = areaBuffer.contents().bindMemory(to: half.self, capacity: count)

            // Step 3: Store data into the buffer
            func store(childIndex cid: Int, parentId pid: Int, rigidBody: ArticulatedRigidBody, climbers: [ArticulatedRigidBody]) {
                let id = index[rigidBody]!

                childCount[id] = UInt8(rigidBody.childJoints.count)
                childIndex[id] = UInt8(cid)
                parentId[id] = UInt16(pid)
                climberCount[id] = UInt8(climbers.count)

                for rigidBody in climbers + [rigidBody] {
                    let id = index[rigidBody]!

                    mass[id] = max(.leastNormalMagnitude, Half(rigidBody.mass))
                    pivot[id] = packed_half3(rigidBody.pivot)
                    localPivot[id] = packed_half3(rigidBody.localPivot)
                    force[id] = packed_half3(rigidBody.force)
                    torque[id] = packed_half3(rigidBody.torque)
                    centerOfMass[id] = packed_half3(rigidBody.centerOfMass)
                    orientation[id] = simd_quath(rigidBody.orientation)
                    inertiaTensor[id] = InertiaTensor(rigidBody.inertiaTensor)
                    localInertiaTensor[id] = packed_float3(rigidBody.localInertiaTensor.diagonal)
                    switch rigidBody.shape {
                    case let .internode(area: a, length: _, radius: _):
                        area[id] = Half(a)
                    case let .leaf(area: a):
                        area[id] = Half(a)
                    case nil: ()
                    }
                    if let parentJoint = rigidBody.parentJoint {
                        jointDamping[id] = max(.leastNormalMagnitude, Half(parentJoint.damping))
                        jointStiffness[id] = max(.leastNormalMagnitude, Half(parentJoint.stiffness))
                        jointOrientation[id] = simd_quath(parentJoint.orientation)
                        localJointPosition[id] = packed_half3(parentJoint.localPosition)
                        localJointOrientation[id] = simd_quath(parentJoint.localOrientation)
                    }
                }
            }

            for level in levels {
                for unitOfWork in level {
                    store(childIndex: unitOfWork.childIndex, parentId: unitOfWork.parentId, rigidBody: unitOfWork.rigidBody, climbers: unitOfWork.climbers)
                }
            }

            // Step 4: Allocated the private buffers
            self.childCountBuffer = device.makeBuffer(length: childCountBuffer.length, options: [.storageModeShared])!
            self.childIndexBuffer = device.makeBuffer(length: childIndexBuffer.length, options: [.storageModeShared])!
            self.parentIdBuffer = device.makeBuffer(length: parentIdBuffer.length, options: [.storageModeShared])!
            self.climberCountBuffer = device.makeBuffer(length: climberCountBuffer.length, options: [.storageModeShared])!
            self.massBuffer = device.makeBuffer(length: massBuffer.length, options: [.storageModeShared])!
            self.pivotBuffer = device.makeBuffer(length: pivotBuffer.length, options: [.storageModeShared])!
            self.localPivotBuffer = device.makeBuffer(length: localPivotBuffer.length, options: [.storageModeShared])!
            self.forceBuffer = device.makeBuffer(length: forceBuffer.length, options: [.storageModeShared])!
            self.torqueBuffer = device.makeBuffer(length: torqueBuffer.length, options: [.storageModeShared])!
            self.centerOfMassBuffer = device.makeBuffer(length: centerOfMassBuffer.length, options: [.storageModeShared])!
            self.orientationBuffer = device.makeBuffer(length: orientationBuffer.length, options: [.storageModeShared])!
            self.inertiaTensorBuffer = device.makeBuffer(length: inertiaTensorBuffer.length, options: [.storageModeShared])!
            self.localInertiaTensorBuffer = device.makeBuffer(length: localInertiaTensorBuffer.length, options: [.storageModeShared])!
            self.jointDampingBuffer = device.makeBuffer(length: jointDampingBuffer.length, options: [.storageModeShared])!
            self.jointStiffnessBuffer = device.makeBuffer(length: jointStiffnessBuffer.length, options: [.storageModeShared])!
            self.localJointPositionBuffer = device.makeBuffer(length: localJointPositionBuffer.length, options: [.storageModeShared])!
            self.localJointOrientationBuffer = device.makeBuffer(length: localJointOrientationBuffer.length, options: [.storageModeShared])!
            self.jointOrientationBuffer = device.makeBuffer(length: jointOrientationBuffer.length, options: [.storageModeShared])!
            self.velocityBuffer = device.makeBuffer(length: velocityBuffer.length, options: [.storageModeShared])!
            self.angularVelocityBuffer = device.makeBuffer(length: angularVelocityBuffer.length, options: [.storageModeShared])!
            self.accelerationBuffer = device.makeBuffer(length: accelerationBuffer.length, options: [.storageModeShared])!
            self.angularAccelerationBuffer = device.makeBuffer(length: angularAccelerationBuffer.length, options: [.storageModeShared])!
            self.areaBuffer = device.makeBuffer(length: areaBuffer.length, options: [.storageModeShared])!

            // Step 5: Blit (copy) from the shared to private buffers
            let commandQueue = device.makeCommandQueue()!
            let commandBuffer = commandQueue.makeCommandBuffer()!
            let blitCommandEncoder = commandBuffer.makeBlitCommandEncoder()!

            blitCommandEncoder.copy(from: childCountBuffer, sourceOffset: 0, to: self.childCountBuffer, destinationOffset: 0, size: childCountBuffer.length)
            blitCommandEncoder.copy(from: childIndexBuffer, sourceOffset: 0, to: self.childIndexBuffer, destinationOffset: 0, size: childIndexBuffer.length)
            blitCommandEncoder.copy(from: parentIdBuffer, sourceOffset: 0, to: self.parentIdBuffer, destinationOffset: 0, size: parentIdBuffer.length)
            blitCommandEncoder.copy(from: climberCountBuffer, sourceOffset: 0, to: self.climberCountBuffer, destinationOffset: 0, size: climberCountBuffer.length)
            blitCommandEncoder.copy(from: massBuffer, sourceOffset: 0, to: self.massBuffer, destinationOffset: 0, size: massBuffer.length)
            blitCommandEncoder.copy(from: pivotBuffer, sourceOffset: 0, to: self.pivotBuffer, destinationOffset: 0, size: pivotBuffer.length)
            blitCommandEncoder.copy(from: localPivotBuffer, sourceOffset: 0, to: self.localPivotBuffer, destinationOffset: 0, size: localPivotBuffer.length)
            blitCommandEncoder.copy(from: forceBuffer, sourceOffset: 0, to: self.forceBuffer, destinationOffset: 0, size: forceBuffer.length)
            blitCommandEncoder.copy(from: torqueBuffer, sourceOffset: 0, to: self.torqueBuffer, destinationOffset: 0, size: torqueBuffer.length)
            blitCommandEncoder.copy(from: centerOfMassBuffer, sourceOffset: 0, to: self.centerOfMassBuffer, destinationOffset: 0, size: centerOfMassBuffer.length)
            blitCommandEncoder.copy(from: orientationBuffer, sourceOffset: 0, to: self.orientationBuffer, destinationOffset: 0, size: orientationBuffer.length)
            blitCommandEncoder.copy(from: inertiaTensorBuffer, sourceOffset: 0, to: self.inertiaTensorBuffer, destinationOffset: 0, size: inertiaTensorBuffer.length)
            blitCommandEncoder.copy(from: localInertiaTensorBuffer, sourceOffset: 0, to: self.localInertiaTensorBuffer, destinationOffset: 0, size: localInertiaTensorBuffer.length)
            blitCommandEncoder.copy(from: jointDampingBuffer, sourceOffset: 0, to: self.jointDampingBuffer, destinationOffset: 0, size: jointDampingBuffer.length)
            blitCommandEncoder.copy(from: jointStiffnessBuffer, sourceOffset: 0, to: self.jointStiffnessBuffer, destinationOffset: 0, size: jointStiffnessBuffer.length)
            blitCommandEncoder.copy(from: localJointPositionBuffer, sourceOffset: 0, to: self.localJointPositionBuffer, destinationOffset: 0, size: localJointPositionBuffer.length)
            blitCommandEncoder.copy(from: localJointOrientationBuffer, sourceOffset: 0, to: self.localJointOrientationBuffer, destinationOffset: 0, size: localJointOrientationBuffer.length)
            blitCommandEncoder.copy(from: jointOrientationBuffer, sourceOffset: 0, to: self.jointOrientationBuffer, destinationOffset: 0, size: jointOrientationBuffer.length)
            blitCommandEncoder.copy(from: velocityBuffer, sourceOffset: 0, to: self.velocityBuffer, destinationOffset: 0, size: velocityBuffer.length)
            blitCommandEncoder.copy(from: angularVelocityBuffer, sourceOffset: 0, to: self.angularVelocityBuffer, destinationOffset: 0, size: angularVelocityBuffer.length)
            blitCommandEncoder.copy(from: accelerationBuffer, sourceOffset: 0, to: self.accelerationBuffer, destinationOffset: 0, size: accelerationBuffer.length)
            blitCommandEncoder.copy(from: angularAccelerationBuffer, sourceOffset: 0, to: self.angularAccelerationBuffer, destinationOffset: 0, size: angularAccelerationBuffer.length)
            blitCommandEncoder.copy(from: areaBuffer, sourceOffset: 0, to: self.areaBuffer, destinationOffset: 0, size: areaBuffer.length)

            blitCommandEncoder.endEncoding()
            commandBuffer.commit()
            commandBuffer.waitUntilCompleted()
        }
    }

    final class CompositeBodies {
        let maxChildCount: Int
        let massBuffer, forceBuffer, torqueBuffer, pivotBuffer, centerOfMassBuffer, inertiaTensorBuffer: MTLBuffer

        init(device: MTLDevice, maxChildCount: Int, ranges: [Range<Int>]) {
            self.maxChildCount = maxChildCount

            var count = 0
            for range in ranges[0..<ranges.count-1] {
                count += maxChildCount * range.count
            }

            self.massBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModeShared])!
            self.forceBuffer = device.makeBuffer(length: count * MemoryLayout<packed_half3>.stride, options: [.storageModeShared])!
            self.torqueBuffer = device.makeBuffer(length: count * MemoryLayout<packed_half3>.stride, options: [.storageModeShared])!
            self.pivotBuffer = device.makeBuffer(length: count * MemoryLayout<packed_half3>.stride, options: [.storageModeShared])!
            self.centerOfMassBuffer = device.makeBuffer(length: count * MemoryLayout<packed_half3>.stride, options: [.storageModeShared])!
            self.inertiaTensorBuffer = device.makeBuffer(length: count * MemoryLayout<InertiaTensor>.stride, options: [.storageModeShared])!
        }
    }

    final class Joints {
        let count: Int
        let inertiaTensorBuffer, torqueBuffer, thetaBuffer: MTLBuffer

        init(device: MTLDevice, count: Int) {
            self.count = count
            self.thetaBuffer = device.makeBuffer(length: 3 * count * MemoryLayout<packed_half3>.stride, options: [.storageModeShared])!
            self.inertiaTensorBuffer = device.makeBuffer(length: count * MemoryLayout<InertiaTensor>.stride, options: [.storageModeShared])!
            self.torqueBuffer = device.makeBuffer(length: count * MemoryLayout<packed_half3>.stride, options: [.storageModeShared])!
        }
    }

    final class PhysicsFields {
        let device: MTLDevice
        let count: Int
        let physicsFieldBuffer: MTLBuffer

        init(device: MTLDevice, fields: [PhysicsField]) {
            self.device = device
            self.count = 1

            let physicsFieldBuffer = device.makeBuffer(length: count * MemoryLayout<ShaderTypes.PhysicsField>.stride, options: [.storageModeShared])!

            let physicsField = physicsFieldBuffer.contents().bindMemory(to: ShaderTypes.PhysicsField.self, capacity: count)

            for (i, field) in fields.enumerated() {
                var shaderField = ShaderTypes.PhysicsField()
                shaderField.position = packed_half3(field.position)
                shaderField.halfExtent = packed_half3(field.halfExtent)
                switch field {
                case let windField as WindField:
                    shaderField.type = .wind
                    let wind = ShaderTypes.WindField(
                        windVelocity: packed_half3(windField.windVelocity),
                        airResistanceMultiplier: Half(windField.airResistanceMultiplier),
                        phi: Half(windField.phi),
                        leafScale: Half(windField.leafScale),
                        branchScale: Half(windField.branchScale),
                        airDensity: Half(windField.airDensity),
                        normal2tangentialDragCoefficientRatio: Half(windField.normal2tangentialDragCoefficientRatio))
                    shaderField.wind = wind
                case let gravityField as GravityField:
                    shaderField.type = .gravity
                    let gravity = ShaderTypes.GravityField(g: packed_half3(gravityField.g))
                    shaderField.gravity = gravity
                default: fatalError("Unsupported Field Type: \(type(of: shaderField))")
                }
                physicsField[i] = shaderField
            }

            self.physicsFieldBuffer = device.makeBuffer(length: physicsFieldBuffer.length, options: [.storageModeShared])!

            let commandQueue = device.makeCommandQueue()!
            let commandBuffer = commandQueue.makeCommandBuffer()!
            let blitCommandEncoder = commandBuffer.makeBlitCommandEncoder()!

            blitCommandEncoder.copy(from: physicsFieldBuffer, sourceOffset: 0, to: self.physicsFieldBuffer, destinationOffset: 0, size: physicsFieldBuffer.length)

            blitCommandEncoder.endEncoding()
            commandBuffer.commit()
            commandBuffer.waitUntilCompleted()

        }
    }

    let rigidBodies: RigidBodies
    let compositeBodies: CompositeBodies
    let joints: Joints
    let physicsFields: PhysicsFields

    public init(device: MTLDevice, root: ArticulatedRigidBody, fields: [PhysicsField]) {
        self.rigidBodies = RigidBodies(device: device, root: root)
        self.compositeBodies = CompositeBodies(device: device, maxChildCount: rigidBodies.maxChildCount, ranges: rigidBodies.ranges)
        self.joints = Joints(device: device, count: rigidBodies.count)
        self.physicsFields = PhysicsFields(device: device, fields: fields)
    }

    func assertValid(otherwise: ((Int, String) -> ()) = { (_, msg) in fatalError(msg) }) {
        for id in 1..<rigidBodies.count {
            var `break`: String? = nil
            joints[id].assertValid { e in
                `break` = e
            }
            rigidBodies[id].assertValid { e in
                `break` = e
            }
            if let `break` = `break` {
                print("Assertion failed for Joint[\(id)]=\(joints[id])")
                print("Assertion failed for RigidBody[\(id)]=\(rigidBodies[id])")
                otherwise(id, `break`)

                break
            }
        }
    }
}

extension MemoryLayoutManager.RigidBodies {
    struct Struct {
        let pivot: simd_float3
        let centerOfMass: simd_float3
        let orientation: simd_quatf
        let inertiaTensor: float3x3

        func assertValid(otherwise: ((String) -> ())) {
            assert(pivot.isFinite, otherwise: otherwise)
            assert(centerOfMass.isFinite, otherwise: otherwise)
            assert(orientation.isFinite, otherwise: otherwise)
            assert(inertiaTensor.isFinite, otherwise: otherwise)
            assert(inertiaTensor.determinant > 0, otherwise: otherwise)
        }
    }

    subscript(id: Int) -> Struct {
        return Struct(
            pivot: simd_float3(pivot[id]),
            centerOfMass: simd_float3(centerOfMass[id]),
            orientation: simd_quatf(orientation[id]),
            inertiaTensor: float3x3(inertiaTensor[id]))
    }

    subscript(rigidBody: RigidBody) -> Struct {
        let id = index[rigidBody]!
        return self[id]
    }

    var mass: UnsafeMutablePointer<Float> {
        massBuffer.contents().bindMemory(to: Float.self, capacity: count)
    }

    var pivot: UnsafeMutablePointer<packed_half3> {
        pivotBuffer.contents().bindMemory(to: packed_half3.self, capacity: count)
    }
    var centerOfMass: UnsafeMutablePointer<packed_half3> {
        centerOfMassBuffer.contents().bindMemory(to: packed_half3.self, capacity: count)
    }
    var orientation: UnsafeMutablePointer<simd_quath> {
        orientationBuffer.contents().bindMemory(to: simd_quath.self, capacity: count)
    }
    var inertiaTensor: UnsafeMutablePointer<InertiaTensor> {
        inertiaTensorBuffer.contents().bindMemory(to: InertiaTensor.self, capacity: count)
    }
}

extension MemoryLayoutManager.Joints {
    struct Struct {
        let torque: simd_float3
        let inertiaTensor: float3x3
        let theta: float3x3

        func assertValid(otherwise: ((String) -> ())) {
            assert(torque.isFinite, "Bad joint torque: \(torque)", otherwise: otherwise)
            assert(theta.isFinite, "Bad joint theta: \(theta)", otherwise: otherwise)
            assert(inertiaTensor.isFinite && inertiaTensor.determinant > 0, "Bad inertiaTensor: \(inertiaTensor) \(inertiaTensor.isFinite) \(inertiaTensor.determinant)", otherwise: otherwise)
            assert(inertiaTensor.cholesky.isFinite, "Bad cholesky for inertia tensor \(inertiaTensor) and determinant \(inertiaTensor.determinant)", otherwise: otherwise)
        }
    }

    subscript(id: Int) -> Struct {
        return Struct(
            torque: simd_float3(torque[id]),
            inertiaTensor: float3x3(inertiaTensor[id]),
            theta: simd_float3x3(
                simd_float3(theta[id*3+0]),
                simd_float3(theta[id*3+1]),
                simd_float3(theta[id*3+2])))
    }

    var theta: UnsafeMutablePointer<packed_half3> {
        return thetaBuffer.contents().bindMemory(to: packed_half3.self, capacity: count*3)
    }
    var inertiaTensor: UnsafeMutablePointer<InertiaTensor> {
        inertiaTensorBuffer.contents().bindMemory(to: InertiaTensor.self, capacity: count)
    }
    var torque: UnsafeMutablePointer<packed_half3> {
        return torqueBuffer.contents().bindMemory(to: packed_half3.self, capacity: count)
    }
}
