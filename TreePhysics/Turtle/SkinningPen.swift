import Foundation
import SceneKit

typealias Indices = [UInt16]

final class SkinningPen: Pen {
    typealias T = (Indices, RigidBody)

    let cylinderPen: CylinderPen
    let rigidBodyPen: RigidBodyPen
    weak var parent: SkinningPen?
    private(set) var bones: [T] = []

    init(cylinderPen: CylinderPen, rigidBodyPen: RigidBodyPen, parent: SkinningPen? = nil) {
        self.cylinderPen = cylinderPen
        self.rigidBodyPen = rigidBodyPen
        self.parent = parent
    }

    func start(at: float2, thickness: Float) {
        cylinderPen.start(at: at, thickness: thickness)
        rigidBodyPen.start(at: at, thickness: thickness)
    }

    func cont(distance: Float, tangent: float2, thickness: Float) -> T {
        let vertices = cylinderPen.cont(distance: distance, tangent: tangent, thickness: thickness)
        let rigidBody = rigidBodyPen.cont(distance: distance, tangent: tangent, thickness: thickness)
        return addBone((vertices, rigidBody))
    }

    func addBone(_ bone: T) -> T {
        if let parent = parent {
            return parent.addBone(bone)
        } else {
            bones.append(bone)
            return bone
        }
    }

    var branch: SkinningPen {
        return SkinningPen(cylinderPen: cylinderPen.branch, rigidBodyPen: rigidBodyPen.branch, parent: self)
    }
}
