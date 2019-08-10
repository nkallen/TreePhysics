import Foundation
import XCTest
@testable import TreePhysics
import SceneKit

class RigidBodyTests: XCTestCase {
    func testFlattenIsBreadthFirst() {
        let root = RigidBody()
        let b1 = RigidBody()
        let b2 = RigidBody()
        let b3 = RigidBody()
        let b4 = RigidBody()
        let b5 = RigidBody()
        let b6 = RigidBody()
        let b7 = RigidBody()
        root.add(b1)
        root.add(b2)
        b1.add(b3)
        b1.add(b4)
        b4.add(b5)
        b2.add(b6)
        b2.add(b7)

        XCTAssertEqual(root.flattened(),
                       [root, b1, b2, b3, b4, b6, b7, b5])
    }
    
    func testLevelsSimple() {
        let root = RigidBody()
        let b1 = RigidBody()
        let b2 = RigidBody()
        root.add(b1)
        b1.add(b2)

        let expected: [Level] = [
            [UnitOfWork(rigidBody: b2, climbers: [b1, root])],
        ]
        XCTAssertEqual(expected, root.levels())
    }

    func testLevelsComplex() {
        let root = RigidBody()
        let b1 = RigidBody()
        let b2 = RigidBody()
        let b3 = RigidBody()
        let b4 = RigidBody()
        let b5 = RigidBody()
        let b6 = RigidBody()
        let b7 = RigidBody()
        let b8 = RigidBody()
        let b9 = RigidBody()

        root.add(b1)

        b1.add(b2)
        b1.add(b3)

        b3.add(b4)
        b4.add(b5)

        b5.add(b6)
        b5.add(b7)

        b7.add(b8)
        b7.add(b9)

        let empty: [RigidBody] = []

        let expected: [Level] = [
            [
                UnitOfWork(rigidBody: b9, climbers: empty),
                UnitOfWork(rigidBody: b8, climbers: empty),
                UnitOfWork(rigidBody: b6, climbers: empty),
                UnitOfWork(rigidBody: b2, climbers: empty)],
            [UnitOfWork(rigidBody: b7, climbers: empty)],
            [UnitOfWork(rigidBody: b5, climbers: [b4, b3])],
            [UnitOfWork(rigidBody: b1, climbers: [root])]
        ]
        XCTAssertEqual(expected, root.levels())
    }
}

/**
 root
  |
  b1
 /  \
b2  b3
    |
    b4
    |
    b5
   / \
  b6 b7
     /\
    b8 b9
 */
//
//[root] -> [b1]
//[b3] -> [b4, b5]
//[b7]
//[b2 b6 b8 b9]
//
//[b1] [b5] [b7] [b2 b6 b8 b9]

extension UnitOfWork: Equatable {
    public static func == (lhs: UnitOfWork, rhs: UnitOfWork) -> Bool {
        return lhs.rigidBody == rhs.rigidBody && lhs.climbers == rhs.climbers
    }
}
