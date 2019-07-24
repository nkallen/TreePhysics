import Foundation
import XCTest
@testable import TreePhysics
import simd

final class FakePen: Pen {
    typealias T = ()

    var points: [float3] = []
    let _branch: FakePen?

    init(branch: FakePen? = nil) {
        self._branch = branch
    }

    func start(at: float3, thickness: Float) {
        points.append(at)
    }

    func cont(distance: Float, tangent: float3, thickness: Float) -> () {
        points.append(points.last! + distance * tangent)
    }

    var branch: FakePen { return _branch! }
}

class TurtleTests: XCTestCase {
    var pen: FakePen!
    var interpreter: Interpreter<FakePen>!
    var stepSize: Float!

    override func setUp() {
        super.setUp()

        self.pen = FakePen(branch: FakePen())
        let configuration = Interpreter<FakePen>.Configuration(angle: .pi / 4)
        self.interpreter = Interpreter(configuration: configuration, pen: self.pen)
        self.stepSize = interpreter.configuration.stepSize
    }

    func testForward() {
        interpreter.interpret([.forward(distance: nil, width: nil), .turnLeft(radians: nil), .forward(distance: nil, width: nil)])
        XCTAssertEqual([float3.zero, float3(0, stepSize, 0), float3(-stepSize * 1.0/sqrt(2), stepSize + stepSize * 1.0/sqrt(2), 0)],
                       pen.points,
                       accuracy: 0.0001)
    }

    func testBranch() {
        interpreter.interpret([.forward(distance: nil, width: nil),
                               .push,
                               .turnRight(radians: nil),
                               .forward(distance: nil, width: nil),
                               .pop,
                               .turnLeft(radians: nil), .forward(distance: nil, width: nil)])
        XCTAssertEqual([
            float3.zero,
            float3(0, stepSize, 0),
            float3(-stepSize * 1.0/sqrt(2), stepSize + stepSize * 1.0/sqrt(2), 0)],
                       pen.points,
                       accuracy: 0.0001)
        XCTAssertEqual([
            float3(0, stepSize, 0),
            float3(stepSize * 1.0/sqrt(2), stepSize + stepSize * 1.0/sqrt(2), 0)],
                       pen.branch.points,
                       accuracy: 0.0001)

    }
}
