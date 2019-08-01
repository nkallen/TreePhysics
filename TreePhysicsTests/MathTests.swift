import XCTest
@testable import TreePhysics
import simd

class MathTests: XCTestCase {
    func testCholesky() {
        XCTAssertEqual(
            float3x3(columns: (
                float3(2, 6, -8),
                float3(0, 1, 5),
                float3(0, 0, 3)
            )),
            float3x3(columns: (
                float3(4,12,-16),
                float3(12,37,-43),
                float3(-16,-43,98)
            )).cholesky)
    }

    func testEigenvaluesAnalytical() {
        let matrix = float3x3(columns: (
            float3(2,1,0),
            float3(1,2,1),
            float3(0,1,2)))
        guard let (eigenvalues, eigenvectors) = matrix.eigen_analytical else {
            XCTFail()
            return
        }

        XCTAssertEqual(
            float3(2 + sqrt(2), 2 - sqrt(2.0), 2),
            eigenvalues, accuracy: 0.0001)

        XCTAssertEqual(
            float3x3(columns: (
                float3(1.0/2, sqrt(2.0) / 2, 1.0/2),
                float3(1.0 / 2, -sqrt(2.0) / 2, 1.0 / 2),
                float3(1 / sqrt(2.0), 0, -1/sqrt(2.0))
            )),
            eigenvectors, accuracy: 0.0001)
    }

    func testTridiagonal() {
        let matrix = double3x3(columns: (
            double3(2,1,0),
            double3(1,2,1),
            double3(0,1,2)))

        let (Q, d, e) = matrix.tridiagonal
        let Q_transpose = Q.transpose
        let X = double3x3(columns: (
            double3(d[0],e[0],0),
            double3(e[0],d[1],e[1]),
            double3(0,e[1],d[2])))
        let Y = Q * X * Q_transpose

        XCTAssertEqual(matrix, Y)
    }

    func testEigenQL_0() {
        let matrix = double3x3(columns: (
            double3(2,1,0),
            double3(1,2,1),
            double3(0,1,2)))

        guard let (eigenvalues, eigenvectors) = matrix.eigen_ql else {
            XCTFail()
            return
        }

        XCTAssertEqual(
            double3(2 - sqrt(2), 2, 2 + sqrt(2.0)),
            eigenvalues, accuracy: 0.0001)

        XCTAssertEqual(
            double3x3(columns: (
                -double3(1.0 / 2, -sqrt(2.0) / 2, 1.0 / 2),
                double3(1 / sqrt(2.0), 0, -1/sqrt(2.0)),
                double3(1.0/2, sqrt(2.0) / 2, 1.0/2)
            )),
            eigenvectors, accuracy: 0.001)
    }

    func testEigenQL_1() {
        let matrix = float3x3(columns: (
            float3(1.0/3,-1.9254154e-10,2.9612822e-08),
            float3(-1.9254154e-10,0.48408476,-0.048982114),
            float3(2.9612824e-08,-0.04898209,0.34924862)
        ))

        let (Q, d, e) = double3x3(matrix).tridiagonal
        XCTAssertEqual(double3x3(columns: (
            double3(1,0,0),
            double3(0,-0.006502,0.999979),
            double3(0,0.999979,0.006502)
            )), Q, accuracy: 0.001)
        XCTAssertEqual(double3(1/3,0.349891,0.483442), d, accuracy: 0.0001)
        XCTAssertEqual(double2(0,-0.049855), e, accuracy: 0.0001)
        XCTAssertEqual(
            float3(1/3,1/3,1/2),
            matrix.eigen_ql!.0, accuracy: 0.0001)
    }

    func testRotation() {
        XCTAssertEqual(
            matrix4x4_rotation(radians: .pi/4, axis: .z),
            matrix4x4_rotation(rotation: float3(0, 0, .pi/4)),
            accuracy: 0.0001)
    }

    func testFoo() {
        let mass: Float = 1
        let length: Float = 1
        let radius: Float = 1
        let momentOfInertiaAboutY = 1.0/12 * mass * length * length // Moment of Inertia of a rod about its center of mass
        let momentOfInertiaAboutX = 1.0/4 * mass * radius * radius // MoI of a disc about its center
        let momentOfInertiaAboutZ = 1.0/4 * mass * radius * radius // ditto

        let inertiaTensor_local = float3x3.init(diagonal:
            float3(momentOfInertiaAboutY + momentOfInertiaAboutX,   // 1/3
                   momentOfInertiaAboutZ + momentOfInertiaAboutX,   // 1/2
                   momentOfInertiaAboutX + momentOfInertiaAboutY))  // 1/3

        XCTAssertEqual(
            float3(1/3,1/2,1/3),
            inertiaTensor_local.eigen_ql!.0
        )

        XCTAssertEqual(
            float3x3(columns: (
                float3(1,0,0),
                float3(0,1,0),
                float3(0,0,1)
            )),
            inertiaTensor_local.eigen_ql!.1
        )


        let rotation = float3x3(rows: [ [9.999998e-01, -1.283656e-07, -4.005872e-08],
                                  [1.949253e-07, 9.510549e-01, 3.090166e-01],
                                  [6.743591e-08, -3.090164e-01, 9.510551e-01] ])

//        XCTAssertEqual(rotation.inverse, rotation.transpose, accuracy: 0.0001)
//        XCTAssertEqual(rotation * float3(0,1,0), float3(-1.283656e-07, 0.9510549, -0.3090164))
//        print(simd_length(float3(-1.283656e-07, 0.9510549, -0.3090164)))

//        let inertiaTensor_local = float3x3([ [3.333333e-01, 0.000000e+00, 0.000000e+00],
//                                             [0.000000e+00, 5.000000e-01, 0.000000e+00],
//                                             [0.000000e+00, 0.000000e+00, 3.333333e-01] ])


//        print(rotation.determinant)
        let inertiaTensor_world = rotation * inertiaTensor_local * rotation.transpose

//        XCTAssertEqual(float3x3([0.333333224, -0.000000000192569516, 0.0000000296128295],
//                                [-0.000000000192569516, 0.484083146, -0.0489818752],
//                                [0.0000000296128313, -0.0489818752, 0.349247545]),
//                       inertiaTensor_world, accuracy: 0.0001)
        print("-----", float3x3(columns: (
            float3(1.0/3,-1.9254154e-10,2.9612822e-08),
            float3(-1.9254154e-10,0.48408476,-0.048982114),
            float3(2.9612824e-08,-0.04898209,0.34924862)
        )).eigen_ql!.0)

        print(float3x3(columns: (
            float3(1.0/3,0,0),
            float3(0,0.48408476,-0.048982114),
            float3(0,-0.04898209,0.34924862)
        )))
        print(inertiaTensor_world)

        print("+++++", inertiaTensor_world.eigen_ql!.0)

        XCTAssertEqual(
            float3(1/3, 1/2, 1/3),
            inertiaTensor_world.eigen_ql!.0, accuracy: 0.0001)

        XCTAssertEqual(
            float3x3([[1.0, -9.087677e-10, 1.0390838e-09], [8.9517993e-10, 1.0151788, -1.1607491], [3.7865955e-19, 153.80026, -11827.249]]),
            inertiaTensor_world.eigen_ql!.1
        )
    }
}
