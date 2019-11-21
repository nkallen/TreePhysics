import Foundation
import simd

public protocol AutoTreeShadowGrid: class {
    subscript(position: simd_float3) -> Float { get set }
}

extension AutoTree {
    public typealias ShadowGrid = AutoTreeShadowGrid

    public final class ArrayBackedShadowGrid: ShadowGrid {
        let config: Config
        public var size: Int
        public var storage: [Float]

        public init(_ config: Config) {
            self.config = config
            self.size = config.initialShadowGridSize
            self.storage = [Float](repeating: 0, count: size*size*size)
        }

        public subscript(position: simd_float3) -> Float {
            get {
                let key = self.key(for: position)
                guard key.x < size && key.y < size && key.z < size else { return 0 }
                return storage[key.x*size*size + key.y*size + key.z]
            }

            set {
                while length(position / config.internodeLength) + sqrt(3*sqr(Float(config.shadowDepth))) > Float(size / 2) {
                    resize()
                }
                let key = self.key(for: position)
                let oldValue = storage[key.x*size*size + key.y*size + key.z]
                let delta = newValue - oldValue
                var q = 0
                while q < config.shadowDepth {
                    var p = -q
                    while p <= q {
                        var s = -q
                        while s <= q {
                            let offset = SIMD3<Int>(p,-q,s)
                            let decayedDelta = delta * pow(config.shadowDecayFactor, Float(q))
                            let k = key &+ offset
                            storage[k.x*size*size + k.y*size + k.z] += decayedDelta
                            s += 1
                        }
                        p += 1
                    }
                    q += 1
                }
            }
        }

        private func key(for position: simd_float3) -> SIMD3<Int> {
            var cell = SIMD3<Int>(floor(position / config.internodeLength))
            cell = cell &+ size / 2
            return cell
        }

        private func resize() {
            let oldStorage = storage
            let oldSize = size
            let newSize = oldSize * 2
            var newStorage = [Float](repeating: 0, count: newSize*newSize*newSize)
            for i in 0..<oldSize {
                for j in 0..<oldSize {
                    for k in 0..<oldSize {
                        let value = oldStorage[i*oldSize*oldSize + j*oldSize + k]
                        let rebase = -oldSize/2 + newSize/2
                        let ii = i + rebase, jj = j + rebase, kk = k + rebase
                        newStorage[ii*newSize*newSize + jj*newSize + kk] = value
                    }
                }
            }
            self.size = newSize
            self.storage = newStorage
        }
    }
}
