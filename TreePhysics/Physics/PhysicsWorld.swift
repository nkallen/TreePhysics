import Foundation

public class PhysicsWorld {
    public init() {}

    private(set) var rigidBodies: Set<RigidBody> = []
    private(set) var rigidBodiesLevelOrder: [ArticulatedRigidBody] = []
    private(set) var fields: Set<PhysicsField> = []
    var rigidBodiesUnordered: [ArticulatedRigidBody] { return rigidBodiesLevelOrder }
    var rigidBodiesLevelOrderReversed: [ArticulatedRigidBody] { return rigidBodiesLevelOrder.reversed() }

    public func add(rigidBody: RigidBody) {
        rigidBodies.insert(rigidBody)
        switch rigidBody {
        case let rigidBody as ArticulatedRigidBody:
            self.rigidBodiesLevelOrder.append(contentsOf: rigidBody.flattened())
        default: ()
        }
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

    func free(articulatedBody: ArticulatedRigidBody) {
        articulatedBody.removeFromParent()
        rigidBodies.insert(articulatedBody)
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
