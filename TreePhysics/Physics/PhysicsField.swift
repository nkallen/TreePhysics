import Foundation
import simd
import SceneKit

protocol PhysicsField {
    var position: float3 { get }
    var halfExtent: float3? { get }
    var `struct`: PhysicsFieldStruct { get }
    func eval(rigidBody: RigidBody,time: TimeInterval) -> float3
}

extension PhysicsField {
    func applies(to position: float3) -> Bool {
        guard let halfExtent = halfExtent else { return true }

        return position.in(min: self.position - halfExtent, max: self.position + halfExtent)
    }
}

final class GravityField: PhysicsField {
    var g: float3
    let position: float3 = float3.zero
    let halfExtent: float3? = nil

    init(_ g: float3 = float3.zero) {
        self.g = g
    }

    func eval(rigidBody: RigidBody, time: TimeInterval) -> float3 {
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

    func eval(rigidBody: RigidBody, time: TimeInterval) -> float3 {
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
    let root: SCNNode
    var index: [int2:SCNNode] = [:]

    init(root: SCNNode) {
        self.root = root
    }

    let gridScale: Float = 2
    let magnitudeTimeScale: Float = 0.05
    let amplitude: Float = 0.005

    func eval(rigidBody: RigidBody, time: TimeInterval) -> float3 {
        let t: Float = magnitudeTimeScale*Float(time)
        let gridPosition = floor(rigidBody.position * gridScale)

        let key = int2(gridPosition.xz)
        let node: SCNNode
        if let value = index[key] {
            node = value
        } else {
            node = createVectorNode(length: 0.25, thickness: 0.5)
            root.addChildNode(node)
            index[key] = node
        }
        node.simdWorldPosition = gridPosition / gridScale * float3(1,0,1)

        let magnitude = fbm(rigidBody.position.xz + t)
        let direction = float3(random(gridPosition.x),random(gridPosition.xz),random(gridPosition.z))
        let DcrossJ = cross(direction, .j)


        guard length(direction) > 10e-10 else { return float3.zero }
        guard length(DcrossJ) > 10e-10 else { return magnitude * direction }

        let s = normalize(DcrossJ)
        let vp = direction + h(length(gridPosition - (dot(gridPosition, .j)) * .j))*s*sin(Float(time))

        let rotation = simd_quatf(from: .j, to: vp)
        node.simdOrientation = rotation
        node.simdScale = float3(1,magnitude*500,1)

        return magnitude * normalize(vp)
    }

    func h(_ t: Float) -> Float {
        let x0: Float = 0.1
        let x1: Float = 0
        let x2: Float = 0
        let x3: Float = 1
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
