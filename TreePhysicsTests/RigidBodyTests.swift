import Foundation
import XCTest
@testable import TreePhysics
import SceneKit

class RigidBodyTests: XCTestCase {

    func testFlattenIsBreadthFirst() {
        let root = Tree.internode()
        let b1 = Tree.internode()
        let b2 = Tree.internode()
        let b3 = Tree.internode()
        let b4 = Tree.internode()
        let b5 = Tree.internode()
        let b6 = Tree.internode()
        let b7 = Tree.internode()
        _ = root.add(b1)
        _ = root.add(b2)
        _ = b1.add(b3)
        _ = b1.add(b4)
        _ = b4.add(b5)
        _ = b2.add(b6)
        _ = b2.add(b7)

        XCTAssertEqual(root.flattened(),
                       [root, b1, b2, b3, b4, b6, b7, b5])
    }
    
    func testLevelsSimple() {
        let root = Tree.internode()
        let b1 = Tree.internode()
        let b2 = Tree.internode()
        _ = root.add(b1)
        _ = b1.add(b2)

        let expected = Levels(rigidBodies: [[[root]]], climbers: [[b1], [b2]])
        XCTAssertEqual(expected, root.levels2())
    }

    func testLevelsComplex() {
        let root = Tree.internode()
        let b1 = Tree.internode()
        let b2 = Tree.internode()
        let b3 = Tree.internode()
        let b4 = Tree.internode()
        let b5 = Tree.internode()
        let b6 = Tree.internode()
        let b7 = Tree.internode()
        let b8 = Tree.internode()
        let b9 = Tree.internode()

        _ = root.add(b1)
        _ = b1.add(b2)
        _ = b1.add(b3)
        _ = b3.add(b4)
        _ = b4.add(b5)
        _ = b5.add(b6)
        _ = b5.add(b7)
        _ = b7.add(b8)
        _ = b7.add(b9)

        let expected = Levels(rigidBodies: [], climbers: [])
        XCTAssertEqual(expected, root.levels2())
    }
}

/**
 root                -> [[b1]]
  |
  b1                 -> [[b2], [b3]]
 /  \
b2  b3               ->
    |
    b4
    |
    b5
   / \
  b6 b7
     /\
    b8 b9
 */

extension UnitOfWork: Equatable {
    public static func == (lhs: UnitOfWork, rhs: UnitOfWork) -> Bool {
        return lhs.rigidBody == rhs.rigidBody && lhs.climbers == rhs.climbers
    }
}
