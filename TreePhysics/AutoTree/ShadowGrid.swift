import Foundation
import simd

public protocol AutoTreeShadowGrid: class {
    subscript(position: SIMD3<Float>) -> Float { get set }
}

extension AutoTree {
    // FIXME
    struct ShadowGridConfig {
        let cellSize: Float
        let decayFactor: Float = 2
        let shadowDepth: Int = 4 // 4..8
    }

    public typealias ShadowGrid = AutoTreeShadowGrid

    final class HashingShadowGrid: ShadowGrid {
        let config: ShadowGridConfig
        private var storage: [SIMD3<Int32>:Float] = [:]

        init(_ config: ShadowGridConfig) {
            self.config = config
        }

        subscript(position: SIMD3<Float>) -> Float {
            get {
                return storage[key(for: position)] ?? 0
            }

            set {
                let oldValue = storage[key(for: position)] ?? 0
                let delta = newValue - oldValue
                for q: Int32 in 0...Int32(config.shadowDepth) {
                    for p: Int32 in -q...q {
                        for s: Int32 in -q...q {
                            let offset = SIMD3<Int32>(p,-q,s)
                            let decayedDelta = delta * pow(config.decayFactor, -Float(q))
                            let key = self.key(for: position, offset: offset)
                            storage[key, default: 0] += decayedDelta
                        }
                    }
                }
            }
        }

        private func key(for position: SIMD3<Float>, offset: SIMD3<Int32> = SIMD3<Int32>.zero) -> SIMD3<Int32> {
            let cell = SIMD3<Int32>(position / config.cellSize)
            return cell &+ offset
        }
    }
}
