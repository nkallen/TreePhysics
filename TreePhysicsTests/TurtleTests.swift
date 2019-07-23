import Foundation
import XCTest
@testable import TreePhysics
import simd

class FakePen: Pen {
    var points: [float2] = []
    let _branch: FakePen?

    init(branch: FakePen? = nil) {
        self._branch = branch
    }

    func start(at: float2, thickness: Float) {
        points.append(at)
    }

    func cont(distance: Float, tangent: float2, thickness: Float) {
        points.append(points.last! + distance * tangent)
    }

    var branch: Pen { return _branch! }
}

class TurtleTests: XCTestCase {
    var pen: FakePen!
    var interpreter: Interpreter!
    var stepSize: Float!

    override func setUp() {
        super.setUp()

        self.pen = FakePen(branch: FakePen())
        let configuration = Interpreter.Configuration(angle: .pi / 4)
        self.interpreter = Interpreter(configuration: configuration, pen: self.pen)
        self.stepSize = interpreter.configuration.stepSize
    }

    func testForward() {
        interpreter.interpret([.forward(distance: nil, width: nil), .turnLeft(radians: nil), .forward(distance: nil, width: nil)])
        XCTAssertEqual([float2.zero, float2(0, stepSize), float2(-stepSize * 1.0/sqrt(2), stepSize + stepSize * 1.0/sqrt(2))],
                       pen.points)
    }

    func testBranch() {
        interpreter.interpret([.forward(distance: nil, width: nil),
                               .push,
                               .turnRight(radians: nil),
                               .forward(distance: nil, width: nil),
                               .pop,
                               .turnLeft(radians: nil), .forward(distance: nil, width: nil)])
        XCTAssertEqual([
            float2.zero,
            float2(0, stepSize),
            float2(-stepSize * 1.0/sqrt(2), stepSize + stepSize * 1.0/sqrt(2))],
                       pen.points)
        XCTAssertEqual([
            float2(0, stepSize),
            float2(stepSize * 1.0/sqrt(2), stepSize + stepSize * 1.0/sqrt(2))],
                       (pen.branch as! FakePen).points)

    }
}
