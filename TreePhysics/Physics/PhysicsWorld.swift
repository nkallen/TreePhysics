import Foundation

public class PhysicsWorld {
    public init() {}

    private(set) var rigidBodies: Set<RigidBody> = []
    private(set) var rigidBodiesLevelOrder: [RigidBody] = []
    private(set) var fields: Set<PhysicsField> = []

    func add(rigidBody: RigidBody) {
        rigidBodies.insert(rigidBody)
        self.rigidBodiesLevelOrder.append(contentsOf: rigidBody.flattened())
    }

    public func add(field: PhysicsField) {
        fields.insert(field)
    }

    func remove(field: PhysicsField) {
        fields.remove(field)
    }

    func remove(rigidBody: RigidBody) {
        rigidBodies.remove(rigidBody)
    }

    func free(articulatedBody: RigidBody) {

    }
}

extension PhysicsField: Equatable, Hashable {
    public static func == (lhs: PhysicsField, rhs: PhysicsField) -> Bool {
        return lhs === rhs
    }

    public func hash(into hasher: inout Hasher) {
        hasher.combine(ObjectIdentifier(self))
    }
}

extension RigidBody: Hashable {
    public static func == (lhs: RigidBody, rhs: RigidBody) -> Bool {
        return lhs === rhs
    }

    public func hash(into hasher: inout Hasher) {
        hasher.combine(ObjectIdentifier(self))
    }
}
