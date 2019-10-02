import Foundation
import simd

protocol HasPosition {
    var position: float3 { get }
}

class LocalitySensitiveHash<T> where T: HasPosition {
    let gridSize: Float

    private var storage: [int3:NSMutableSet] = [:]

    init(gridSize: Float) {
        self.gridSize = gridSize
    }

    func add(_ t: T) {
        if let set = storage[key(for: t.position)] {
            set.add(t)
        } else {
            let set = NSMutableSet(array: [t])
            storage[key(for: t.position)] = set
        }
    }

    func elements(near position: float3) -> [T] {
        var result: [T] = []
        for i in -1...1 {
            for j in -1...1 {
                for k in -1...1 {
                    let offset = int3(Int32(i), Int32(j), Int32(k))
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

    private func key(for position: float3, offset: int3 = int3.zero) -> int3 {
        let cell = int3(position / gridSize)
        print(cell &+ offset)
        return cell &+ offset
    }
}
