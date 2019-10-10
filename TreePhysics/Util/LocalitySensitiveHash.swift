import Foundation
import simd

protocol HasPosition {
    var position: SIMD3<Float> { get }
}

class LocalitySensitiveHash<T> where T: HasPosition {
    let cellSize: Float

    private var storage: [SIMD3<Int32>:NSMutableSet] = [:]

    init(cellSize: Float) {
        self.cellSize = cellSize
    }

    func add(_ t: T) {
        if let set = storage[key(for: t.position)] {
            set.add(t)
        } else {
            let set = NSMutableSet(array: [t])
            storage[key(for: t.position)] = set
        }
    }

    func remove(_ t: T) {
        if let set = storage[key(for: t.position)] {
            set.remove(t)
        }
    }

    func elements(near position: SIMD3<Float>) -> [T] {
        var result: [T] = []
        for i in -1...1 {
            for j in -1...1 {
                for k in -1...1 {
                    let offset = SIMD3<Int32>(Int32(i), Int32(j), Int32(k))
                    if let set = storage[key(for: position, offset: offset)] {
                        for object in set {
                            result.append(object as! T)
                        }
                    }
                }
            }
        }
        return result
    }

    private func key(for position: SIMD3<Float>, offset: SIMD3<Int32> = SIMD3<Int32>.zero) -> SIMD3<Int32> {
        let cell = SIMD3<Int32>(position / cellSize)
        return cell &+ offset
    }
}
