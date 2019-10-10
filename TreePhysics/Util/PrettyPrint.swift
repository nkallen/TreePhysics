import Foundation
import simd
import ShaderTypes

extension Joint: CustomDebugStringConvertible {
    public var debugDescription: String {
        return "Joint \(parentRigidBody.name): k=\(stiffness), θ=\(θ)"
    }
}

extension JointStruct: CustomDebugStringConvertible {
    public var debugDescription: String {
        return "Joint: θ=\(θ)"
    }
}

extension RigidBody: CustomDebugStringConvertible {
    public var debugDescription: String {
        return "RigidBody \(name): kind=\(kind), rotation=\(rotation), mass=\(mass), centerOfMass=\(centerOfMass), force=\(force), torque=\(torque), inertiaTensor=\(inertiaTensor)"
    }
}

extension RigidBodyStruct: CustomDebugStringConvertible {
    public var debugDescription: String {
        return "RigidBody: parentId=\(parentId) position=\(position), rotation=\(rotation), mass=\(mass), length=\(length), radius=\(radius), centerOfMass=\(centerOfMass), force=\(force), torque=\(torque), inertiaTensor=\(inertiaTensor)"
    }
}

extension CompositeBody: CustomDebugStringConvertible {
    public var debugDescription: String {
        return "Composite Body: mass=\(mass), inertiaTensor=\(inertiaTensor), force=\(force), torque=\(torque), centerOfMass=\(centerOfMass)"
    }
}

extension CompositeBodyStruct: CustomDebugStringConvertible {
    public var debugDescription: String {
        return "CompositeBodyStruct(position: \(position), mass: \(mass), inertiaTensor: \(inertiaTensor), force: \(force), torque: \(torque), centerOfMass: \(centerOfMass))"
    }
}

extension AutoTree.Node: CustomDebugStringConvertible {
    var debugDescription: String {
        return "AutoTree.\(type(of: self))(generation: \(generation ?? -1) position: \(position) orientation: \(orientation))"
    }
}
