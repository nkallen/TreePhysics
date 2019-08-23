import Foundation

extension Joint: CustomDebugStringConvertible {
    var debugDescription: String {
        return "Joint \(parentRigidBody.name): k=\(k), θ=\(θ)"
    }
}

extension JointStruct: CustomDebugStringConvertible {
    public var debugDescription: String {
        return "Joint: θ=\(float3x3(θ))"
    }
}

extension RigidBody: CustomDebugStringConvertible {
    var debugDescription: String {
        return "RigidBody \(name): position=\(position), rotation=\(rotation), mass=\(mass), length=\(length), radius=\(radius), centerOfMass=\(centerOfMass), force=\(force), torque=\(torque), inertiaTensor=\(inertiaTensor)"
    }
}

extension RigidBodyStruct: CustomDebugStringConvertible {
    public var debugDescription: String {
        return "RigidBody: parentId=\(parentId) position=\(float3(position)), rotation=\(float3x3(rotation)), mass=\(mass), length=\(length), radius=\(radius), centerOfMass=\(float3(centerOfMass)), force=\(float3(force)), torque=\(float3(torque)), inertiaTensor=\(float3x3(inertiaTensor))"
    }
}

extension CompositeBody: CustomDebugStringConvertible {
    var debugDescription: String {
        return "Composite Body: mass=\(mass), inertiaTensor=\(inertiaTensor), force=\(force), torque=\(torque), centerOfMass=\(centerOfMass)"
    }
}

extension CompositeBodyStruct: CustomDebugStringConvertible {
    public var debugDescription: String {
        return "CompositeBodyStruct(position: \(float3(position)), mass: \(mass), inertiaTensor: \(inertiaTensor), force: \(float3(force)), torque: \(float3(torque)), centerOfMass: \(float3(centerOfMass)))"
    }
}
