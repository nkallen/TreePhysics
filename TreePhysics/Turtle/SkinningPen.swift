import Foundation
import SceneKit

class SkinningPen: Pen {
    let cylinderPen: CylinderPen
    let rigidBodyPen: RigidBodyPen

    init(cylinderPen: CylinderPen, rigidBodyPen: RigidBodyPen) {
        self.cylinderPen = cylinderPen
        self.rigidBodyPen = rigidBodyPen
    }

    func start(at: float2, thickness: Float) {
        cylinderPen.start(at: at, thickness: thickness)
        rigidBodyPen.start(at: at, thickness: thickness)
    }

    func cont(distance: Float, tangent: float2, thickness: Float) {
        cylinderPen.cont(distance: distance, tangent: tangent, thickness: thickness)
        rigidBodyPen.cont(distance: distance, tangent: tangent, thickness: thickness)
    }

    var branch: Pen {
        return SkinningPen(cylinderPen: cylinderPen.branch as! CylinderPen, rigidBodyPen: rigidBodyPen.branch as! RigidBodyPen)
    }
}
