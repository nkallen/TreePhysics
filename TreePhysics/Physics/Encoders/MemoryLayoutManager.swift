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
        let maxRangeWidth: Int
        let count: Int
        let childCountBuffer, firstChildIdBuffer, childIndexBuffer, parentIdBuffer, climberCountBuffer, massBuffer, pivotBuffer, localPivotBuffer, forceBuffer, torqueBuffer, centerOfMassBuffer, orientationBuffer, inertiaTensorBuffer, localInertiaTensorBuffer, jointDampingBuffer, jointStiffnessBuffer, localJointPositionBuffer, localJointOrientationBuffer, jointOrientationBuffer, velocityBuffer, angularVelocityBuffer, accelerationBuffer, angularAccelerationBuffer, angularMomentumBuffer, areaBuffer, shapeBuffer: MTLBuffer

        init(device: MTLDevice, root: ArticulatedRigidBody) {
            var ranges: [Range<Int>] = []
            let levels = root.levels()
            var offset = 0
            var id = 0
            var index = [RigidBody:Int]()

            // Step 1: Determine the buffer memory layout (i.e., the index and the ranges of work)
            var maxClimberCount = 0
            var maxChildCount = 0
            var maxRangeWidth = 0
            for level in levels {
                for unitOfWork in level {
                    index[unitOfWork.rigidBody] = id
                    id += 1
                    maxClimberCount = max(maxClimberCount, unitOfWork.climbers.count)
                    maxChildCount = max(maxChildCount, unitOfWork.rigidBody.childJoints.count)
                }

                let range = offset..<(offset+level.count)
                ranges.append(range)
                maxRangeWidth = max(maxRangeWidth, level.count)
                offset += level.count
            }
            self.ranges = ranges
            self.maxClimberCount = maxClimberCount
            self.maxChildCount = maxChildCount
            self.maxRangeWidth = maxRangeWidth

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
            let firstChildIdBuffer = device.makeBuffer(length: count * MemoryLayout<UInt16>.stride, options: [.storageModeShared])!
            let childIndexBuffer = device.makeBuffer(length: count * MemoryLayout<UInt8>.stride, options: [.storageModeShared])!
            let parentIdBuffer = device.makeBuffer(length: count * MemoryLayout<UInt32>.stride, options: [.storageModeShared])!
            let climberCountBuffer = device.makeBuffer(length: count * MemoryLayout<UInt8>.stride, options: [.storageModeShared])!
            let massBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModeShared])!
            let pivotBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            let localPivotBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            let forceBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            let torqueBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            let centerOfMassBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            let orientationBuffer = device.makeBuffer(length: count * MemoryLayout<simd_quatf>.stride, options: [.storageModeShared])!
            let inertiaTensorBuffer = device.makeBuffer(length: count * MemoryLayout<InertiaTensor>.stride, options: [.storageModeShared])!
            let localInertiaTensorBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            let jointDampingBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModeShared])!
            let jointStiffnessBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModeShared])!
            let localJointPositionBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            let localJointOrientationBuffer = device.makeBuffer(length: count * MemoryLayout<simd_quath>.stride, options: [.storageModeShared])!
            let jointOrientationBuffer = device.makeBuffer(length: count * MemoryLayout<simd_quath>.stride, options: [.storageModeShared])!
            let velocityBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            let angularVelocityBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            let accelerationBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            let angularAccelerationBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            let angularMomentumBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            let areaBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModeShared])!
            let shapeBuffer = device.makeBuffer(length: count * MemoryLayout<UInt8>.stride, options: [.storageModeShared])!

            let childCount = childCountBuffer.contents().bindMemory(to: UInt8.self, capacity: count)
            let firstChildId = firstChildIdBuffer.contents().bindMemory(to: UInt16.self, capacity: count)
            let childIndex = childIndexBuffer.contents().bindMemory(to: UInt8.self, capacity: count)
            let parentId = parentIdBuffer.contents().bindMemory(to: UInt32.self, capacity: count)
            let climberCount = climberCountBuffer.contents().bindMemory(to: UInt8.self, capacity: count)
            let mass = massBuffer.contents().bindMemory(to: Float.self, capacity: count)
            let pivot = pivotBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
            let localPivot = localPivotBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
            let force = forceBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
            let torque = torqueBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
            let centerOfMass = centerOfMassBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
            let orientation = orientationBuffer.contents().bindMemory(to: simd_quatf.self, capacity: count)
            let inertiaTensor = inertiaTensorBuffer.contents().bindMemory(to: InertiaTensor.self, capacity: count)
            let localInertiaTensor = localInertiaTensorBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
            let jointDamping = jointDampingBuffer.contents().bindMemory(to: Float.self, capacity: count)
            let jointStiffness = jointStiffnessBuffer.contents().bindMemory(to: Float.self, capacity: count)
            let localJointPosition = localJointPositionBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
            let localJointOrientation = localJointOrientationBuffer.contents().bindMemory(to: simd_quath.self, capacity: count)
            let jointOrientation = jointOrientationBuffer.contents().bindMemory(to: simd_quath.self, capacity: count)
            let velocity = velocityBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
            let angularVelocity = angularVelocityBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
            let acceleration = accelerationBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
            let angularAcceleration = angularAccelerationBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
            let angularMomentum = angularMomentumBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
            let area = areaBuffer.contents().bindMemory(to: Float.self, capacity: count)
            let shape = shapeBuffer.contents().bindMemory(to: UInt8.self, capacity: count)

            // Step 3: Store data into the buffer
            func store(_ unitOfWork: UnitOfWork) {
                let cid = unitOfWork.childIndex
                let pid = unitOfWork.parentId
                let rigidBody = unitOfWork.rigidBody
                let climbers = unitOfWork.climbers

                assert(pid != NO_PARENT)
                let id = index[rigidBody]!

                childCount[id] = UInt8(rigidBody.childJoints.count)
                if let fid = unitOfWork.firstChildId {
                    firstChildId[id] = UInt16(fid)
                }
                childIndex[id] = UInt8(cid)
                if let parentJoint = rigidBody.parentJoint {
                    parentId[id] = UInt32(index[parentJoint.parentRigidBody]!)
                } else {
                    parentId[id] = UInt32(NO_PARENT)
                }
                climberCount[id] = UInt8(climbers.count)

                for rigidBody in climbers + [rigidBody] {
                    let id = index[rigidBody]!

                    mass[id] = max(.leastNormalMagnitude, Float(rigidBody.mass))
                    pivot[id] = packed_float3(rigidBody.pivot)
                    localPivot[id] = packed_float3(rigidBody.localPivot)
                    force[id] = packed_float3(rigidBody.force)
                    torque[id] = packed_float3(rigidBody.torque)
                    centerOfMass[id] = packed_float3(rigidBody.centerOfMass)
                    orientation[id] = rigidBody.orientation
                    inertiaTensor[id] = InertiaTensor(rigidBody.inertiaTensor)
                    localInertiaTensor[id] = packed_float3(rigidBody.localInertiaTensor.diagonal)
                    switch rigidBody.shape {
                    case let .internode(area: a, length: _, radius: _):
                        shape[id] = ShapeType.internode.rawValue
                        area[id] = Float(a)
                    case let .leaf(area: a):
                        shape[id] = ShapeType.leaf.rawValue
                        area[id] = Float(a)
                    case nil: ()
                    }
                    if let parentJoint = rigidBody.parentJoint {
                        jointDamping[id] = max(.leastNormalMagnitude, Float(parentJoint.damping))
                        jointStiffness[id] = max(.leastNormalMagnitude, Float(parentJoint.stiffness))
                        jointOrientation[id] = simd_quath(parentJoint.orientation)
                        localJointPosition[id] = packed_float3(parentJoint.localPosition)
                        localJointOrientation[id] = simd_quath(parentJoint.localOrientation)
                    }
                }
            }

            for level in levels {
                for unitOfWork in level {
                    store(unitOfWork)
                }
            }

            // Step 4: Allocated the private buffers
            self.childCountBuffer = device.makeBuffer(length: childCountBuffer.length, options: [.storageModeShared])!
            self.firstChildIdBuffer = device.makeBuffer(length: firstChildIdBuffer.length, options: [.storageModeShared])!
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
            self.angularMomentumBuffer = device.makeBuffer(length: angularMomentumBuffer.length, options: [.storageModeShared])!
            self.areaBuffer = device.makeBuffer(length: areaBuffer.length, options: [.storageModeShared])!
            self.shapeBuffer = device.makeBuffer(length: shapeBuffer.length, options: [.storageModeShared])!

            self.childCountBuffer.label = "Child count"
            self.firstChildIdBuffer.label = "First child id"
            self.childIndexBuffer.label = "Child index"
            self.parentIdBuffer.label = "Parent id"
            self.climberCountBuffer.label = "Climber count"
            self.massBuffer.label = "Mass"
            self.pivotBuffer.label = "Pivot"
            self.localPivotBuffer.label = "Local pivot"
            self.forceBuffer.label = "Force"
            self.torqueBuffer.label = "Torque"
            self.centerOfMassBuffer.label = "Center of mass"
            self.orientationBuffer.label = "Orientation"
            self.inertiaTensorBuffer.label = "Inertia tensor"
            self.localInertiaTensorBuffer.label = "Local inertia tensor"
            self.jointDampingBuffer.label = "Joint damping"
            self.jointStiffnessBuffer.label = "Joint stiffness"
            self.localJointPositionBuffer.label = "Local joint position"
            self.localJointOrientationBuffer.label = "Local joint orientation"
            self.jointOrientationBuffer.label = "Joint orientation"
            self.velocityBuffer.label = "Velocity"
            self.angularVelocityBuffer.label = "Angular velocity"
            self.accelerationBuffer.label = "Acceleration"
            self.angularAccelerationBuffer.label = "Angular acceleration"
            self.angularMomentumBuffer.label = "Angular momentum"
            self.areaBuffer.label = "Area"
            self.shapeBuffer.label = "Shape"

            // Step 5: Blit (copy) from the shared to private buffers
            let commandQueue = device.makeCommandQueue()!
            let commandBuffer = commandQueue.makeCommandBuffer()!
            let blitCommandEncoder = commandBuffer.makeBlitCommandEncoder()!

            blitCommandEncoder.copy(from: childCountBuffer, sourceOffset: 0, to: self.childCountBuffer, destinationOffset: 0, size: childCountBuffer.length)
            blitCommandEncoder.copy(from: firstChildIdBuffer, sourceOffset: 0, to: self.firstChildIdBuffer, destinationOffset: 0, size: firstChildIdBuffer.length)
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
            blitCommandEncoder.copy(from: angularMomentumBuffer, sourceOffset: 0, to: self.angularMomentumBuffer, destinationOffset: 0, size: angularMomentumBuffer.length)
            blitCommandEncoder.copy(from: areaBuffer, sourceOffset: 0, to: self.areaBuffer, destinationOffset: 0, size: areaBuffer.length)
            blitCommandEncoder.copy(from: shapeBuffer, sourceOffset: 0, to: self.shapeBuffer, destinationOffset: 0, size: shapeBuffer.length)

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
            self.forceBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            self.torqueBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            self.pivotBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            self.centerOfMassBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            self.inertiaTensorBuffer = device.makeBuffer(length: count * MemoryLayout<InertiaTensor>.stride, options: [.storageModeShared])!

            self.massBuffer.label = "Composite mass"
            self.forceBuffer.label = "Composite force"
            self.torqueBuffer.label = "Composite torque"
            self.pivotBuffer.label = "Composite pivot"
            self.centerOfMassBuffer.label = "Composite center of mass"
            self.inertiaTensorBuffer.label = "Composite inertia tensor"
        }

    }

    final class FreeBodies {
        let toBeFreedCountBuffer, countBuffer, toBeFreedIndexBuffer, indexBuffer, massBuffer, forceBuffer, torqueBuffer, inertiaTensorBuffer: MTLBuffer

        init(device: MTLDevice, count: Int) {
            self.toBeFreedCountBuffer = device.makeBuffer(length: 1 * MemoryLayout<UInt32>.stride, options: [.storageModeShared])!
            self.countBuffer = device.makeBuffer(length: 1 * MemoryLayout<UInt32>.stride, options: [.storageModeShared])!
            self.toBeFreedIndexBuffer = device.makeBuffer(length: count * MemoryLayout<UInt32>.stride, options: [.storageModeShared])!
            self.indexBuffer = device.makeBuffer(length: count * MemoryLayout<UInt32>.stride, options: [.storageModeShared])!
            self.massBuffer = device.makeBuffer(length: count * MemoryLayout<Float>.stride, options: [.storageModeShared])!
            self.forceBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            self.torqueBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            self.inertiaTensorBuffer = device.makeBuffer(length: count * MemoryLayout<InertiaTensor>.stride, options: [.storageModeShared])!

            self.toBeFreedCountBuffer.label = "Bodies to be freed"
            self.countBuffer.label = "Free body count"
            self.toBeFreedIndexBuffer.label = "Bodies to be freed index"
            self.indexBuffer.label = "Free body index"
            self.massBuffer.label = "Free body mass"
            self.forceBuffer.label = "Free body force"
            self.torqueBuffer.label = "Free body torque"
            self.inertiaTensorBuffer.label = "Free body inertia tensor"
        }
    }

    final class Joints {
        let count: Int
        let inertiaTensorBuffer, torqueBuffer, thetaBuffer: MTLBuffer

        init(device: MTLDevice, count: Int) {
            self.count = count
            self.thetaBuffer = device.makeBuffer(length: 3 * count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!
            self.inertiaTensorBuffer = device.makeBuffer(length: count * MemoryLayout<InertiaTensor>.stride, options: [.storageModeShared])!
            self.torqueBuffer = device.makeBuffer(length: count * MemoryLayout<packed_float3>.stride, options: [.storageModeShared])!

            self.thetaBuffer.label = "Joint theta"
            self.inertiaTensorBuffer.label = "Joint inertia tensor"
            self.torqueBuffer.label = "Joint torque buffer"
        }
    }

    final class PhysicsFields {
        let device: MTLDevice
        let count: Int
        let physicsFieldBuffer: MTLBuffer

        init(device: MTLDevice, fields: [PhysicsField]) {
            self.device = device
            self.count = fields.count

            let physicsFieldBuffer = device.makeBuffer(length: count * MemoryLayout<ShaderTypes.PhysicsField>.stride, options: [.storageModeShared])!

            let physicsField = physicsFieldBuffer.contents().bindMemory(to: ShaderTypes.PhysicsField.self, capacity: count)

            for (i, field) in fields.enumerated() {
                var shaderField = ShaderTypes.PhysicsField()
                shaderField.position = packed_float3(field.position)
                shaderField.halfExtent = packed_float3(field.halfExtent)
                switch field {
                case let windField as WindField:
                    shaderField.type = .wind
                    let wind = ShaderTypes.WindField(
                        windVelocity: packed_float3(windField.windVelocity),
                        airResistanceMultiplier: Float(windField.airResistanceMultiplier),
                        phi: Float(windField.phi),
                        leafScale: Float(windField.leafScale),
                        branchScale: Float(windField.branchScale),
                        airDensity: Float(windField.airDensity),
                        normal2tangentialDragCoefficientRatio: Float(windField.normal2tangentialDragCoefficientRatio))
                    shaderField.wind = wind
                case let gravityField as GravityField:
                    shaderField.type = .gravity
                    let gravity = ShaderTypes.GravityField(g: packed_float3(gravityField.g))
                    shaderField.gravity = gravity
                default: fatalError("Unsupported Field Type: \(type(of: shaderField))")
                }
                physicsField[i] = shaderField
            }

            self.physicsFieldBuffer = device.makeBuffer(length: physicsFieldBuffer.length, options: [.storageModeShared])!
            self.physicsFieldBuffer.label = "Physics field"

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
    let freeBodies: FreeBodies

    public init(device: MTLDevice, root: ArticulatedRigidBody, fields: [PhysicsField]) {
        self.rigidBodies = RigidBodies(device: device, root: root)
        self.compositeBodies = CompositeBodies(device: device, maxChildCount: rigidBodies.maxChildCount, ranges: rigidBodies.ranges)
        self.joints = Joints(device: device, count: rigidBodies.count)
        self.physicsFields = PhysicsFields(device: device, fields: fields)
        self.freeBodies = FreeBodies(device: device, count: rigidBodies.count)
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
        let force: simd_float3
        let torque: simd_float3

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
            orientation: orientation[id],
            inertiaTensor: float3x3(inertiaTensor[id]),
            force: simd_float3(force[id]),
            torque: simd_float3(torque[id]))
    }

    subscript(rigidBody: RigidBody) -> Struct {
        let id = index[rigidBody]!
        return self[id]
    }

    var mass: UnsafeMutablePointer<Float> {
        massBuffer.contents().bindMemory(to: Float.self, capacity: count)
    }
    var pivot: UnsafeMutablePointer<packed_float3> {
        pivotBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
    }
    var centerOfMass: UnsafeMutablePointer<packed_float3> {
        centerOfMassBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
    }
    var orientation: UnsafeMutablePointer<simd_quatf> {
        orientationBuffer.contents().bindMemory(to: simd_quatf.self, capacity: count)
    }
    var inertiaTensor: UnsafeMutablePointer<InertiaTensor> {
        inertiaTensorBuffer.contents().bindMemory(to: InertiaTensor.self, capacity: count)
    }
    var force: UnsafeMutablePointer<packed_float3> {
        forceBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
    }
    var torque: UnsafeMutablePointer<packed_float3> {
        torqueBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
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

    var theta: UnsafeMutablePointer<packed_float3> {
        return thetaBuffer.contents().bindMemory(to: packed_float3.self, capacity: count*3)
    }
    var inertiaTensor: UnsafeMutablePointer<InertiaTensor> {
        inertiaTensorBuffer.contents().bindMemory(to: InertiaTensor.self, capacity: count)
    }
    var torque: UnsafeMutablePointer<packed_float3> {
        return torqueBuffer.contents().bindMemory(to: packed_float3.self, capacity: count)
    }
}

//
//struct RigidBody2: Foo {
//    // sourcery: buffer = "tree"
//    let parentId: Int
//    // sourcery: buffer = "tree"
//    let childCount: Int
//    // sourcery: buffer = "tree"
//    let firstChild: Int
//    // sourcery: buffer = "tree"
//    let childIndex: Int
//
//    // sourcery: buffer = "physics"
//    let mass: Float
//    // sourcery: buffer = "physics"
//    let pivot: simd_float3
//    // sourcery: buffer = "physics"
//    let centerOfMass: simd_float3
//}
//
//protocol Foo {}
