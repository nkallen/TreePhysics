import XCTest
@testable import TreePhysics
import simd

/* These tests aren't unit tests of the actual code so much as demonstrations of the physics and
 * calculations involved in the code. They are put here as a reference, either as documentation of
 * certain laws of physics, or as a model that can be used to debug other things.
 */
class ModelTests: XCTestCase {
    let time: Float = 1.0/60
    let k: Float = 1
    let β: Float = 1

    /* This test shows that the 2d case (a cylinder moving in the xy plane), using a simple
     * moment-of-inertia, is equivalent to a 3d case, where all the objects and forces are again
     * in the xy plane, but using an inertia tensor and generalized eigendecomposition.
     */
    func test2DCaseIsSameAsPlanar3DCase() {
        let root = ArticulatedRigidBody.static()
        let rigidBody = Internode(length: 1, radius: 1, density: 1)
        let parentJoint = root.add(rigidBody, rotation: simd_quatf.identity, position: float3.zero)

        // 1. The 2D setup:
        let momentOfInertia_jointSpace = rigidBody.momentOfInertia + rigidBody.mass * sqr(distance(rigidBody.centerOfMass, parentJoint.position))

        let force = float3(-1, 0, 0)
        let torque = cross(float3(0, 1, 0), force)
        let torque_jointSpace = parentJoint.rotate(vector: torque)
        XCTAssertEqual(torque, torque_jointSpace)

        // angle, angular velocity, angular acceleration:
        let expectedZ = float3(7.557869e-05, 0.009053111, 0.5406928)
        XCTAssertEqual(
            expectedZ,
            evaluateDifferential(a: momentOfInertia_jointSpace, b: β * k, c: k, g: torque_jointSpace.z, y_0: parentJoint.θ[0].z, y_ddt_0: parentJoint.θ[0].z, at: time))

        // 2. A 3D setup, with everything in one 2D plane.
        let pr = parentJoint.rotate(vector: rigidBody.centerOfMass - parentJoint.position)
        let inertiaTensor_jointSpace = parentJoint.rotate(tensor: rigidBody.inertiaTensor) -
            rigidBody.mass * sqr(pr.skew)

        let L = inertiaTensor_jointSpace.cholesky
        let L_inverse = L.inverse, L_transpose_inverse = L_inverse.transpose
        let A = L_inverse * (k * matrix_identity_float3x3) * L_transpose_inverse
        let (Λ, X) = A.eigen_analytical!
        let U = L_transpose_inverse * X
        let U_transpose = U.transpose, U_inverse = U.inverse
        let torque_diagonal = U_transpose * torque_jointSpace
        let θ_diagonal_0 = U_inverse * parentJoint.θ[0]
        let θ_ddt_diagonal_0 = U_inverse * parentJoint.θ[1]
        let βΛ = β * Λ
        let θ_diagonal = float3x3(rows: [
            evaluateDifferential(a: 1, b: βΛ.x, c: Λ.x, g: torque_diagonal.x, y_0: θ_diagonal_0.x, y_ddt_0: θ_ddt_diagonal_0.x, at: time),
            evaluateDifferential(a: 1, b: βΛ.y, c: Λ.y, g: torque_diagonal.y, y_0: θ_diagonal_0.y, y_ddt_0: θ_ddt_diagonal_0.y, at: time),
            evaluateDifferential(a: 1, b: βΛ.z, c: Λ.z, g: torque_diagonal.z, y_0: θ_diagonal_0.z, y_ddt_0: θ_ddt_diagonal_0.z, at: time)])

        let θ = U * θ_diagonal
        XCTAssertEqual(float3.zero, θ.transpose[0])
        XCTAssertEqual(float3.zero, θ.transpose[1])
        XCTAssertEqual(expectedZ, θ.transpose[2], accuracy: 0.0001)
    }
}

extension Internode {
    var momentOfInertia: Float {
        // Moment of Inertia of a rod about its center of mass
        return 1.0/4 * mass * sqr(radius) + 1.0/12 * mass * sqr(length)
    }
}
