import Foundation
import simd
@testable import TreePhysics

public class Emitter {
    let maxAge: TimeInterval
    let simulator: CPUSimulator
    let birthRate: Float

    var bucket: (Float, Int)? = nil

    public var count = 0
    public var particles: [(Leaf, Date)?]
    public var ticks = 0
    let noise = Noise()
    var total = 0

    public init(birthRate: Float, max: Int, maxAge: TimeInterval, simulator: CPUSimulator) {
        precondition(birthRate <= 1)
        
        self.birthRate = birthRate
        self.maxAge = maxAge
        self.particles = [(Leaf, Date)?](repeating: nil, count: max)
        self.simulator = simulator
    }

    public func emit() -> Leaf? {
        ticks += 1
        guard ticks % Int(1/birthRate) == 0 else { return nil }
        guard count + 1 < particles.count else { return nil }

        let leaf = Leaf(length: 1, density: 500)
        let seed = 1 + total
        leaf.rotation =
            simd_quatf(angle: noise.random(seed + 0) * 2 * .pi, axis: .x) *
            simd_quatf(angle: noise.random(seed + 1) * 2 * .pi, axis: .y) *
            simd_quatf(angle: noise.random(seed + 2) * 2 * .pi, axis: .z)

        leaf.rotation = leaf.rotation.normalized
        leaf.node.simdOrientation = leaf.rotation

        particles[count] = (leaf, Date())
        simulator.add(rigidBody: leaf)

        count += 1
        count %= particles.count
        total += 1
        total %= Int.max
        return leaf
    }

    public func update() {
        let now = Date()
        for i in 0..<count {
            let (leaf, createdAt) = particles[i]!
            if abs(createdAt.timeIntervalSince(now)) > maxAge {
                leaf.node.removeFromParentNode()
                particles[i] = particles[count - 1]
                count -= 1
            }
        }
    }
}
