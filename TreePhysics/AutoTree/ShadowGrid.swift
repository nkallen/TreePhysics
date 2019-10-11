import Foundation
import simd

public protocol AutoTreeShadowGrid: class {
    subscript(position: SIMD3<Float>) -> Float { get set }
}

extension AutoTree {
    public typealias ShadowGrid = AutoTreeShadowGrid

    final class HashingShadowGrid: ShadowGrid {
        let config: Config
        private var storage: [SIMD3<Int32>:Float] = [:]

        init(_ config: Config) {
            self.config = config
        }

        subscript(position: SIMD3<Float>) -> Float {
            get {
                return storage[key(for: position), default: 0]
            }

            set {
                let key = self.key(for: position)
                let oldValue = storage[key, default: 0]
                let delta = newValue - oldValue
                var q: Int32 = 0
                while q <= Int32(config.shadowDepth) {
                    var p = -q
                    while p <= q {
                        var s = -q
                        while s <= q {
                            let offset = SIMD3<Int32>(p,-q,s)
                            let decayedDelta = delta * pow(config.shadowDecayFactor, Float(q))
                            storage[key &+ offset, default: 0] += decayedDelta
                            s += 1
                        }
                        p += 1
                    }
                    q += 1
                }
            }
        }

        private func key(for position: SIMD3<Float>, offset: SIMD3<Int32> = SIMD3<Int32>.zero) -> SIMD3<Int32> {
            let cell = SIMD3<Int32>(floor(position / config.internodeLength))
            return cell
        }
    }
}
