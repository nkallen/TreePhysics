import Foundation
import XCTest
@testable import TreePhysics
import SceneKit

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

class RigidBodyTests: XCTestCase {
    func testLevels() {
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

        XCTAssertEqual([[b9, b8, b6, b2], [b7], [b5], [b1]],
            root.levels)
    }
}
