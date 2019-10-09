import Foundation
import simd

protocol AutoTreeShadowGrid: class {
    subscript(position: float3) -> Float { get set }
}

extension AutoTree {
    struct ShadowGridConfig {
        let cellSize: Float
        let decayFactor: Float = 2
        let shadowDepth: Int = 2 // 4..8
    }

    typealias ShadowGrid = AutoTreeShadowGrid

    final class HashingShadowGrid: ShadowGrid {
        let config: ShadowGridConfig
        private var storage: [int3:Float] = [:]

        init(_ config: ShadowGridConfig) {
            self.config = config
        }

        subscript(position: float3) -> Float {
            get {
                return storage[key(for: position)] ?? 0
            }

            set {
                let oldValue = storage[key(for: position)] ?? 0
                let delta = newValue - oldValue
                for q in 0...config.shadowDepth {
                    for p in -q...q {
                        for s in -q...q {
                            let offset = int3(Int32(p),Int32(-q),Int32(s))
                            let decayedDelta = delta * pow(config.decayFactor, -Float(q))
                            let key = self.key(for: position, offset: offset)
                            storage[key] = (storage[key] ?? 0) + decayedDelta
                        }
                    }
                }
            }
        }

        private func key(for position: float3, offset: int3 = int3.zero) -> int3 {
            let cell = int3(position / config.cellSize)
            return cell &+ offset
        }
    }
}
