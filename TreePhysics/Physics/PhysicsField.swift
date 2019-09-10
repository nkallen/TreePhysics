import Foundation
import simd
import SceneKit

protocol PhysicsField {
    var position: float3 { get }
    var halfExtent: float3? { get }
    var `struct`: PhysicsFieldStruct { get }
    func force(rigidBody: Internode, time: TimeInterval) -> float3
    func torque(rigidBody: Internode, time: TimeInterval) -> float3?
}

extension PhysicsField {
    func applies(to position: float3) -> Bool {
        guard let halfExtent = halfExtent else { return true }

        return position.in(min: self.position - halfExtent, max: self.position + halfExtent)
    }

    func torque(rigidBody: Internode, time: TimeInterval) -> float3? {
        return nil
    }
}

final class GravityField: PhysicsField {
    var g: float3
    let position: float3 = float3.zero
    let halfExtent: float3? = nil

    init(_ g: float3 = float3.zero) {
        self.g = g
    }

    func force(rigidBody: Internode, time: TimeInterval) -> float3 {
        return g * rigidBody.mass
    }

    var `struct`: PhysicsFieldStruct {
        return PhysicsFieldStruct(
            position: position,
            halfExtent: halfExtent ?? float3(repeating: -1))
    }
}

final class AttractorField: PhysicsField {
    var position: float3 = float3.zero
    let halfExtent: float3? = float3(1, 1, 1)

    let a: Float = 0.05
    let b: Float = 0.01
    let c: Float = 0.1

    func force(rigidBody: Internode, time: TimeInterval) -> float3 {
        let delta = self.position - rigidBody.position
        let distance = length(delta)
        if (distance > 0) {
            let direction = normalize(delta)
            // NOTE: this is a bell-shaped curve, so objects very far and very near are weakly affected
            let force = direction * a * powf(.e, -sqr(distance - b)/(2*c))
            return force
        } else {
            return float3(repeating: 0)
        }
    }

    var `struct`: PhysicsFieldStruct {
        return PhysicsFieldStruct(
            position: position,
            halfExtent: halfExtent ?? float3(repeating: -1))
    }
}

final class WindField: PhysicsField {
    var position = float3.zero
    var halfExtent: float3? = nil

    let cellSize: Float = 0.5
    let magnitudeTimeScale: Float = 0.1
    let rotationTimeScale: Float = 2
    let amplitude: Float = 0.03

    func force(rigidBody: Internode, time: TimeInterval) -> float3 {
        let magnitudeTime: Float = magnitudeTimeScale*Float(time)
        let rotationTime: Float = rotationTimeScale*Float(time)
        let gridPosition = floor(rigidBody.position / cellSize)

        let magnitude = fbm(rigidBody.position.xz + magnitudeTime)
        var direction = float3(random(gridPosition.x),random(gridPosition.xz),random(gridPosition.z))
        guard length(direction) > 10e-10 else { return float3.zero }
        let DcrossJ = cross(direction, .j)
        direction = normalize(direction)

        guard length(DcrossJ) > 10e-10 else { return magnitude * direction }

        let s = normalize(DcrossJ)
        let vp = normalize(direction + h(length(gridPosition - (dot(gridPosition, .j)) * .j))*s*sin(rotationTime))

        return magnitude * vp
    }

    func h(_ t: Float) -> Float {
        let t = sin(t)
        let x0: Float = 0.1
        let x1: Float = 0
        let x2: Float = 1
        let x3: Float = 0
        return bezier(x0, x1, x2, x3, t: t)
    }

    func random(_ x: Float) -> Float {
        return modf(sin(x)*1.0).1
    }

    func random(_ st: float2) -> Float {
        return modf(sin(dot(st,
                            float2(12.9898,78.233))) *
            43758.5453123).1
    }

    // Based on Morgan McGuire @morgan3d
    // https://www.shadertoy.com/view/4dS3Wd
    func noise(_ st: float2) -> Float {
        let i = floor(st)
        let f = fract(st)

        // Four corners in 2D of a tile
        let a = random(i)
        let b = random(i + float2(1.0, 0.0))
        let c = random(i + float2(0.0, 1.0))
        let d = random(i + float2(1.0, 1.0))

        let u = f * f * (3.0 - 2.0 * f);

        return mix(a, b, t: u.x) +
            (c - a) * u.y * (1.0 - u.x) +
                (d - b) * u.x * u.y;
    }

    let octaves = 6
    func fbm(_ st: float2) -> Float {
        // Initial values
        var st = st
        var value: Float = 0.0
        var amplitude: Float = self.amplitude

        // Loop of octaves
        for _ in 0..<octaves {
            value += amplitude * noise(st)
            st *= 2.0
            amplitude *= 0.5
        }
        return value
    }

    var `struct`: PhysicsFieldStruct {
        return PhysicsFieldStruct(
            position: position,
            halfExtent: halfExtent ?? float3(repeating: -1))
    }

}

final class FieldVisualizer: PhysicsField {
    let underlying: PhysicsField
    let root: SCNNode
    let cellSize: Float
    var index: [int2:SCNNode] = [:]

    init(_ underlying: PhysicsField, root: SCNNode, cellSize: Float = 0.1) {
        self.underlying = underlying
        self.root = root
        self.cellSize = cellSize
    }

    var position: float3 { return underlying.position }
    var halfExtent: float3? { return underlying.halfExtent }
    var `struct`: PhysicsFieldStruct { return underlying.struct}

    func force(rigidBody: Internode, time: TimeInterval) -> float3 {
        let gridPosition = floor(rigidBody.position / cellSize)

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
        node.simdOrientation = rotation
        node.simdScale = float3(1,magnitude * 10,1)

        return result
    }
}
