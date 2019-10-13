import Foundation
import simd

protocol HasPosition {
    var position: SIMD3<Float> { get }
}

final class LocalitySensitiveHash<T> where T: HasPosition, T: Hashable {
    let cellSize: Float

    private var storage: [SIMD3<Int>:Set<T>] = [:]

    init(cellSize: Float) {
        self.cellSize = cellSize
    }

    func add(_ t: T) {
        let key = self.key(for: t.position)
        var set = storage[key, default: []]
        set.insert(t)
        storage[key] = set
    }

    func remove(_ t: T) {
        let key = self.key(for: t.position)
        if var set = storage[key] {
            set.remove(t)
            storage[key] = set
        }
    }

    // returns a FlattenedSequence for performance reasons (since it can be constructed without
    // copying (unlike an array)
    func elements(near position: SIMD3<Float>) -> FlattenSequence<[Set<T>]> {
        var result: [Set<T>] = []
        let key = self.key(for: position)
        for i in -1...1 {
            for j in -1...1 {
                for k in -1...1 {
                    let offset = SIMD3<Int>(i, j, k)
                    if let set = storage[key &+ offset] {
                        result.append(set)
                    }
                }
            }
        }
        return result.joined()
    }

    private func key(for position: SIMD3<Float>) -> SIMD3<Int> {
        return SIMD3<Int>(floor(position / cellSize))
    }
}
