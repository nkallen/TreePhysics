import Foundation
import XCTest
@testable import TreePhysics
import simd

final class FakePen: Pen {
    typealias T = ()

    var points: [SIMD3<Float>] = []
    let _branch: FakePen?

    init(branch: FakePen? = nil) {
        self._branch = branch
    }

    func start(at: SIMD3<Float>, orientation: simd_quatf, thickness: Float) {
        points.append(at)
    }

    func cont(distance: Float, orientation: simd_quatf, thickness: Float) -> () {
        points.append(points.last! + distance * orientation.heading)
    }

    func copy(scale: Float, orientation: simd_quatf) -> () {
    }

    func branch() -> FakePen { return _branch! }
}

class TurtleTests: XCTestCase {
    var pen: FakePen!
    var interpreter: Interpreter<FakePen>!
    var stepSize: Float!

    override func setUp() {
        super.setUp()

        self.pen = FakePen(branch: FakePen())
        let configuration = InterpreterConfig(angle: .pi / 4)
        self.interpreter = Interpreter(configuration: configuration, pen: self.pen)
        self.stepSize = interpreter.configuration.stepSize
    }

    func testForward() {
        interpreter.interpret([.forward(distance: nil, width: nil), .turnLeft(radians: nil), .forward(distance: nil, width: nil)])
        XCTAssertEqual([.zero, SIMD3<Float>(0, stepSize, 0), SIMD3<Float>(stepSize * 1.0/sqrt(2), stepSize + stepSize * 1.0/sqrt(2), 0)],
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
            .zero,
            SIMD3<Float>(0, stepSize, 0),
            SIMD3<Float>(stepSize * 1.0/sqrt(2), stepSize + stepSize * 1.0/sqrt(2), 0)],
                       pen.points,
                       accuracy: 0.0001)
        XCTAssertEqual([
            SIMD3<Float>(0, stepSize, 0),
            SIMD3<Float>(-stepSize * 1.0/sqrt(2), stepSize + stepSize * 1.0/sqrt(2), 0)],
                       pen.branch().points,
                       accuracy: 0.0001)

    }
}
