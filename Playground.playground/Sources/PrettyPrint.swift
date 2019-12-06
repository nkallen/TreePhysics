import Foundation
import TreePhysics


extension AutoTree.ArrayBackedShadowGrid: CustomPlaygroundDisplayConvertible {
    public var playgroundDescription: Any {
        return size
    }
}

extension AutoTree.GrowthSimulator: CustomPlaygroundDisplayConvertible {
    public var playgroundDescription: Any {
        return ""
    }
}
