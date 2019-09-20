import Foundation
import simd
@testable import TreePhysics

public class Emitter {
    let maxAge: TimeInterval
    var count = 0
    var particles: [(Leaf, Date)?]
    let noise = Noise()
    var total = 0

    public init(max: Int, maxAge: TimeInterval) {
        self.maxAge = maxAge
        self.particles = [(Leaf, Date)?](repeating: nil, count: max)
    }

    public func emit() -> Leaf? {
        guard count + 1 < particles.count else { return nil }

        let leaf = Leaf(length: 1, density: 750)
        let seed = total
        leaf.rotation =
            simd_quatf(angle: noise.random(seed + 0) * 2 * .pi, axis: .x) *
            simd_quatf(angle: noise.random(seed + 1) * 2 * .pi, axis: .y) *
            simd_quatf(angle: noise.random(seed + 2) * 2 * .pi, axis: .z)
        leaf.rotation = leaf.rotation.normalized
        leaf.node.simdOrientation = leaf.rotation

        particles[count] = (leaf, Date())

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
