import Foundation
import SceneKit
import Darwin

// FIXME kill this class, not necessary
extension Tree {
    static let K: Float = 200
    static let E: Float = 0.6e9
    static let B: Float = 0.02
    static let maxAngle: Float = Float.pi / 3
    static let minAngle: Float = -Float.pi / 3
}

final class Tree {
    let root: RigidBody

    init(_ root: RigidBody) {
        self.root = root
    }
}
