import Foundation
import simd
import SceneKit
import ShaderTypes

public protocol PhysicsField {
    var position: float3 { get }
    var halfExtent: float3? { get }
    var `struct`: PhysicsFieldStruct { get }
    func force(rigidBody: RigidBody, time: TimeInterval) -> float3
    func torque(rigidBody: RigidBody, time: TimeInterval) -> float3?
}

extension PhysicsField {
    func applies(to position: float3) -> Bool {
        guard let halfExtent = halfExtent else { return true }

        return position.in(min: self.position - halfExtent, max: self.position + halfExtent)
    }
}

public final class FieldVisualizer: PhysicsField {
    let underlying: PhysicsField
    let root: SCNNode
    let cellSize: Float
    var index: [int2:SCNNode] = [:]

    public init(_ underlying: PhysicsField, root: SCNNode, cellSize: Float = 0.1) {
        self.underlying = underlying
        self.root = root
        self.cellSize = cellSize
    }

    public var position: float3 { return underlying.position }
    public var halfExtent: float3? { return underlying.halfExtent }
    public var `struct`: PhysicsFieldStruct { return underlying.struct}

    public func force(rigidBody: RigidBody, time: TimeInterval) -> float3 {
        let gridPosition = floor(rigidBody.centerOfMass / cellSize)

        let key = int2(gridPosition.xz)
        let node: SCNNode
        if let value = index[key] {
            node = value
        } else {
            node = createVectorNode(length: 1, thickness: 0.3)
            root.addChildNode(node)
            index[key] = node
        }
        node.simdWorldPosition = gridPosition * cellSize * float3(1,0,1)

        let result = underlying.force(rigidBody: rigidBody, time: time)
        let magnitude = length(result)

        guard magnitude > 10e-10 else {
            node.simdScale = float3.zero
            return result
        }

        let rotation = simd_quatf(from: .j, to: normalize(result))
        node.simdScale = float3(1, 1, 1)
        node.simdOrientation = rotation

        return result
    }

    public func torque(rigidBody: RigidBody, time: TimeInterval) -> float3? {
        return underlying.torque(rigidBody: rigidBody, time: time)
    }
}
