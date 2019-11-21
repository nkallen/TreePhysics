import Foundation
import simd
import ShaderTypes

extension Joint: CustomDebugStringConvertible {
    public var debugDescription: String {
        return "Joint \(parentRigidBody.name): k=\(stiffness), θ=\(θ)"
    }
}

extension RigidBody: CustomDebugStringConvertible {
    public var debugDescription: String {
        return name
    }
}

extension CompositeBody: CustomDebugStringConvertible {
    public var debugDescription: String {
        return "Composite Body: mass=\(mass), inertiaTensor=\(inertiaTensor), force=\(force), torque=\(torque), centerOfMass=\(centerOfMass)"
    }
}

extension AutoTree.Node: CustomDebugStringConvertible {
    public var debugDescription: String {
        return "AutoTree.\(type(of: self))(generation: \(generation ?? -1) position: \(position) orientation: \(orientation))"
    }
}
