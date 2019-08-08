import Foundation

func XCTAssertEqual(_ a: double2, _ b: double2, accuracy: Double, file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.x, b.x, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.y, b.y, accuracy: accuracy, file: file, line: line)
}

func XCTAssertEqual(_ a: float3, _ b: float3, accuracy: Float, file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.x, b.x, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.y, b.y, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.z, b.z, accuracy: accuracy, file: file, line: line)
}

func XCTAssertEqual(_ a: float4, _ b: float4, accuracy: Float, file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.x, b.x, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.y, b.y, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.z, b.z, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.w, b.w, accuracy: accuracy, file: file, line: line)
}

func XCTAssertEqual(_ a: double3, _ b: double3, accuracy: Double, file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.x, b.x, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.y, b.y, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.z, b.z, accuracy: accuracy, file: file, line: line)
}

func XCTAssertEqual(_ a: float3x3, _ b: float3x3, accuracy: Float, file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.columns.0, b.columns.0, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.columns.1, b.columns.1, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.columns.2, b.columns.2, accuracy: accuracy, file: file, line: line)
}

func XCTAssertEqual(_ a: float4x4, _ b: float4x4, accuracy: Float, file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.columns.0, b.columns.0, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.columns.1, b.columns.1, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.columns.2, b.columns.2, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.columns.3, b.columns.3, accuracy: accuracy, file: file, line: line)
}

func XCTAssertEqual(_ a: double3x3, _ b: double3x3, accuracy: Double, file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.columns.0, b.columns.0, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.columns.1, b.columns.1, accuracy: accuracy, file: file, line: line)
    XCTAssertEqual(a.columns.2, b.columns.2, accuracy: accuracy, file: file, line: line)
}

func XCTAssertEqual(_ a: [float3], _ b: [float3], accuracy: Float, file: StaticString = #file, line: UInt = #line) {
    XCTAssertEqual(a.count, b.count, file: file, line: line)
    for (left, right) in zip(a, b) {
        XCTAssertEqual(left, right, accuracy: accuracy, file: file, line: line)
    }
}

func XCTAssertEqual(_ a: DifferentialSolution, _ b: DifferentialSolution, accuracy: Float, file: StaticString = #file, line: UInt = #line) {
    switch (a, b) {
    case let (.realDistinct(c1_left, c2_left, r1_left, r2_left, k_left),
              .realDistinct(c1_right, c2_right, r1_right, r2_right, k_right)):
        XCTAssertEqual(c1_left, c1_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(c2_left, c2_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(r1_left, r1_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(r2_left, r2_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(k_left, k_right, accuracy: accuracy)
    case let (.real(c1_left, c2_left, r_left, k_left),
              .real(c1_right, c2_right, r_right, k_right)):
        XCTAssertEqual(c1_left, c1_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(c2_left, c2_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(r_left, r_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(k_left, k_right, accuracy: accuracy, file: file, line: line)
    case let (.complex(c1_left, c2_left, lambda_left, mu_left, k_left),
              .complex(c1_right, c2_right, lambda_right, mu_right, k_right)):
        XCTAssertEqual(c1_left, c1_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(c2_left, c2_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(lambda_left, lambda_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(mu_left, mu_right, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(k_left, k_right, accuracy: accuracy, file: file, line: line)
    default:
        XCTFail(file: file, line: line)
    }
}
