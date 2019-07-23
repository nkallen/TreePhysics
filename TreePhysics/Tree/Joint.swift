import Foundation
import simd
import SceneKit

final class Joint: HasTransform {
    unowned let parentRigidBody: RigidBody
    let childRigidBody: RigidBody

    var transform: matrix_float3x3 = matrix_identity_float3x3
    var angularAcceleration: Float = 0
    var angularVelocity: Float = 0
    var angle: Float = 0 {
        didSet {
            node.simdRotation = float4(0, 0, 1, angle)
            self.transform = parentRigidBody.transform * matrix3x3_translation(0, parentRigidBody.length) * matrix3x3_rotation(radians: self.angle)
        }
    }

    let k: Float

    let node: SCNNode

    init(parent: RigidBody, child: RigidBody, k: Float? = nil) {
        self.parentRigidBody = parent
        self.childRigidBody = child
        self.k = k ?? Joint.computeK(radius: parent.radius)

        let node = SCNNode()
        node.addChildNode(child.node)
        node.name = "Joint"
        // SceneKit's cylinder's (0,0) is at its center, so the joint is at length/2 on the y axis
        node.simdPosition = float3(0, parent.length/2, 0)
        self.node = node
        parent.childJoints.append(self)
        child.parentJoint = self
        updateTransform()
    }

    func updateTransform() {
        self.transform = parentRigidBody.transform * matrix3x3_translation(0, parentRigidBody.length) * matrix3x3_rotation(radians: self.angle)
    }

    private static func computeK(radius: Float) -> Float {
        if radius <= 0.003 {
            return Tree.K
        } else if radius <= 0.005 {
            return Tree.K
        } else if radius <= 0.01 {
            return 4 * Tree.K
        } else if radius <= 0.02 {
            return 256 * Tree.K
        } else if radius <= 0.04 {
            return 256 * Tree.K
        } else if radius <= 0.075 {
            return 1024 * Tree.K
        } else if radius <= 0.15 {
            return 4096 * Tree.K
        } else {
            return 16384 * Tree.K
        }
    }
}
