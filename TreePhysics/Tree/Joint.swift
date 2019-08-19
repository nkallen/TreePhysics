import Foundation
import simd
import SceneKit

fileprivate let local_ijk = matrix_float4x4(diagonal: float4(1,1,1,0))

final class Joint: HasTransform {
    unowned let parentRigidBody: RigidBody
    let childRigidBody: RigidBody

    var transform: matrix_float4x4 = matrix_identity_float4x4
    // NOTE: θ[0] is the xyz rotation of the joint; θ[1] is the angular velocity, etc.
    var θ: float3x3 = float3x3(0)
    private(set) var rotation_world2local: float3x3 = matrix_identity_float3x3
    private let translation_local: float4x4

    let k: Float

    init(parent: RigidBody, child: RigidBody, k: Float? = nil) {
        self.parentRigidBody = parent
        self.childRigidBody = child
        self.k = k ?? Joint.computeK(radius: parent.radius, length: parent.length)
        self.translation_local = matrix4x4_translation(0, parentRigidBody.length, 0)
        updateTransform()
    }

    @inline(__always)
    func updateTransform() {
        let eulerAngles = θ[0]
        let rotation_local = matrix4x4_rotation(rotation: eulerAngles)
        self.transform = parentRigidBody.transform * translation_local * rotation_local
        self.rotation_world2local = matrix3x3_rotation(from: local_ijk, to: transform.inverse)
    }

    @inline(__always)
    func rotate(tensor: float3x3) -> float3x3 {
        return rotation_world2local * tensor * rotation_world2local.transpose
    }

    @inline(__always)
    func rotate(vector: float3) -> float3 {
        return rotation_world2local * vector
    }

    private static func computeK(radius: Float, length: Float) -> Float {
        return Tree.K
//        return Tree.E * .pi * radius * radius / length
    }
}
