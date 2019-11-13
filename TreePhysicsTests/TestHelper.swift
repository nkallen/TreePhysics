import Foundation
import XCTest
@testable import TreePhysics
import simd
import ShaderTypes

func XCTAssertEqual(_ a: SIMD2<Double>, _ b: SIMD2<Double>, accuracy: Double, _ message: @autoclosure () -> String = "", file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.x, b.x, accuracy: accuracy, message(), file: file, line: line)
    XCTAssertEqual(a.y, b.y, accuracy: accuracy, message(), file: file, line: line)
}

func XCTAssertEqual(_ a: SIMD3<Float>, _ b: SIMD3<Float>, accuracy: Float, _ message: @autoclosure () -> String = "", file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.x, b.x, accuracy: accuracy, message() + ".x", file: file, line: line)
    XCTAssertEqual(a.y, b.y, accuracy: accuracy, message() + ".y", file: file, line: line)
    XCTAssertEqual(a.z, b.z, accuracy: accuracy, message() + ".z", file: file, line: line)
}

func XCTAssertEqual(_ a: float4, _ b: float4, accuracy: Float, _ message: @autoclosure () -> String = "", file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.x, b.x, accuracy: accuracy, message(), file: file, line: line)
    XCTAssertEqual(a.y, b.y, accuracy: accuracy, message(), file: file, line: line)
    XCTAssertEqual(a.z, b.z, accuracy: accuracy, message(), file: file, line: line)
    XCTAssertEqual(a.w, b.w, accuracy: accuracy, message(), file: file, line: line)
}

func XCTAssertEqual(_ a: SIMD3<Double>, _ b: SIMD3<Double>, accuracy: Double, _ message: @autoclosure () -> String = "", file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.x, b.x, accuracy: accuracy, message(), file: file, line: line)
    XCTAssertEqual(a.y, b.y, accuracy: accuracy, message(), file: file, line: line)
    XCTAssertEqual(a.z, b.z, accuracy: accuracy, message(), file: file, line: line)
}

func XCTAssertEqual(_ a: float3x3, _ b: float3x3, accuracy: Float, _ message: @autoclosure () -> String = "", file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.columns.0, b.columns.0, accuracy: accuracy, message() + ".columns.0", file: file, line: line)
    XCTAssertEqual(a.columns.1, b.columns.1, accuracy: accuracy, message() + ".columns.1", file: file, line: line)
    XCTAssertEqual(a.columns.2, b.columns.2, accuracy: accuracy, message() + ".columns.2", file: file, line: line)
}

func XCTAssertEqual(_ a: simd_quatf, _ b: simd_quatf, accuracy: Float, _ message: @autoclosure () -> String = "", file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.angle, b.angle, accuracy: accuracy, message() + ".angle", file: file, line: line)
    XCTAssertEqual(a.axis, b.axis, accuracy: accuracy, message() + ".axis", file: file, line: line)
}

func XCTAssertEqual(_ a: float4x4, _ b: float4x4, accuracy: Float, _ message: @autoclosure () -> String = "", file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.columns.0, b.columns.0, accuracy: accuracy, message(), file: file, line: line)
    XCTAssertEqual(a.columns.1, b.columns.1, accuracy: accuracy, message(), file: file, line: line)
    XCTAssertEqual(a.columns.2, b.columns.2, accuracy: accuracy, message(), file: file, line: line)
    XCTAssertEqual(a.columns.3, b.columns.3, accuracy: accuracy, message(), file: file, line: line)
}

func XCTAssertEqual(_ a: double3x3, _ b: double3x3, accuracy: Double, _ message: @autoclosure () -> String = "", file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.columns.0, b.columns.0, accuracy: accuracy, message(), file: file, line: line)
    XCTAssertEqual(a.columns.1, b.columns.1, accuracy: accuracy, message(), file: file, line: line)
    XCTAssertEqual(a.columns.2, b.columns.2, accuracy: accuracy, message(), file: file, line: line)
}

func XCTAssertEqual(_ a: [SIMD3<Float>], _ b: [SIMD3<Float>], accuracy: Float, _ message: @autoclosure () -> String = "", file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.count, b.count, file: file, line: line)
    for (left, right) in zip(a, b) {
        XCTAssertEqual(left, right, accuracy: accuracy, message(), file: file, line: line)
    }
}

func XCTAssertEqual(_ a: DifferentialSolution, _ b: DifferentialSolution, accuracy: Float, _ message: @autoclosure () -> String = "", file: StaticString = #file, line: UInt = #line) {
    switch (a, b) {
    case let (.realDistinct(c1_left, c2_left, r1_left, r2_left, k_left),
              .realDistinct(c1_right, c2_right, r1_right, r2_right, k_right)):
        XCTAssertEqual(c1_left, c1_right, accuracy: accuracy, message(), file: file, line: line)
        XCTAssertEqual(c2_left, c2_right, accuracy: accuracy, message(), file: file, line: line)
        XCTAssertEqual(r1_left, r1_right, accuracy: accuracy, message(), file: file, line: line)
        XCTAssertEqual(r2_left, r2_right, accuracy: accuracy, message(), file: file, line: line)
        XCTAssertEqual(k_left, k_right, accuracy: accuracy)
    case let (.real(c1_left, c2_left, r_left, k_left),
              .real(c1_right, c2_right, r_right, k_right)):
        XCTAssertEqual(c1_left, c1_right, accuracy: accuracy, message(), file: file, line: line)
        XCTAssertEqual(c2_left, c2_right, accuracy: accuracy, message(), file: file, line: line)
        XCTAssertEqual(r_left, r_right, accuracy: accuracy, message(), file: file, line: line)
        XCTAssertEqual(k_left, k_right, accuracy: accuracy, message(), file: file, line: line)
    case let (.complex(c1_left, c2_left, lambda_left, mu_left, k_left),
              .complex(c1_right, c2_right, lambda_right, mu_right, k_right)):
        XCTAssertEqual(c1_left, c1_right, accuracy: accuracy, message(), file: file, line: line)
        XCTAssertEqual(c2_left, c2_right, accuracy: accuracy, message(), file: file, line: line)
        XCTAssertEqual(lambda_left, lambda_right, accuracy: accuracy, message(), file: file, line: line)
        XCTAssertEqual(mu_left, mu_right, accuracy: accuracy, message(), file: file, line: line)
        XCTAssertEqual(k_left, k_right, accuracy: accuracy, message(), file: file, line: line)
    default:
        XCTFail(file: file, line: line)
    }
}

func XCTAssertEqual(_ a: CompositeBody, _ b: CompositeBodyStruct, accuracy: Float, _ message: @autoclosure () -> String = "", file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.mass, b.mass, accuracy: accuracy, "mass", file: file, line: line)
    XCTAssertEqual(a.force, b.force, accuracy: accuracy, "force", file: file, line: line)
    XCTAssertEqual(a.torque, b.torque, accuracy: accuracy, "torque", file: file, line: line)
    XCTAssertEqual(a.centerOfMass, b.centerOfMass, accuracy: accuracy, "center of mass", file: file, line: line)
    XCTAssertEqual(a.inertiaTensor, b.inertiaTensor, accuracy: accuracy, "inertia tensor", file: file, line: line)
}

func XCTAssertEqual(_ a: Joint, _ b: JointStruct, accuracy: Float, _ message: @autoclosure () -> String = "", file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.θ, b.θ, accuracy: accuracy, message(), file: file, line: line)
}

func XCTAssertEqual(_ a: ArticulatedRigidBody, _ b: RigidBodyStruct, accuracy: Float, _ message: @autoclosure () -> String = "", file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.mass, b.mass, accuracy: accuracy, "mass", file: file, line: line)
    XCTAssertEqual(a.force, b.force, accuracy: accuracy, "force", file: file, line: line)
    XCTAssertEqual(a.torque, b.torque, accuracy: accuracy, "torque", file: file, line: line)
    XCTAssertEqual(a.centerOfMass, b.centerOfMass, accuracy: accuracy, "center of mass", file: file, line: line)
    XCTAssertEqual(a.inertiaTensor, b.inertiaTensor, accuracy: accuracy, "inertia tensor", file: file, line: line)

    XCTAssertEqual(a.pivot, b.pivot, accuracy: accuracy, "pivot", file: file, line: line)
    XCTAssertEqual(float3x3(a.rotation), b.rotation, accuracy: accuracy, "rotation", file: file, line: line)
}
