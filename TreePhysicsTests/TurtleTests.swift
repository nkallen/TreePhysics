import Foundation
import XCTest
@testable import TreePhysics
import simd

class FakePen: Pen {
    var points: [float2] = []

    func start(at: float2, tangent: float2) {
        points.append(at)
    }

    func cont(at: float2, tangent: float2) {
        points.append(at)
    }

    func end(at: float2, tangent: float2) {
        points.append(at)
    }
}

class TurtleTests: XCTestCase {
    var pens: [FakePen]!
    var interpreter: Interpreter!
    var stepSize: Float!

    override func setUp() {
        super.setUp()
        self.pens = []
        self.interpreter = Interpreter() { () in
            let pen = FakePen()
            self.pens.append(pen)
            return pen
        }
        self.stepSize = interpreter.configuration.stepSize
    }

    func testForward() {
        interpreter.interpret(commands: [.forward(distance: nil, width: nil), .turnLeft(radians: nil), .forward(distance: nil, width: nil)])
        XCTAssertEqual([float2.zero, float2(0, stepSize), float2(stepSize * 1.0/sqrt(2), stepSize + stepSize * 1.0/sqrt(2))],
                       pens.first!.points)
    }

    func testBranch() {
        interpreter.interpret(commands: [.forward(distance: nil, width: nil),
                                         .push,
                                         .turnRight(radians: nil),
                                         .forward(distance: nil, width: nil),
                                         .pop,
                                         .turnLeft(radians: nil), .forward(distance: nil, width: nil)])
        XCTAssertEqual([
            float2.zero,
            float2(0, stepSize),
            float2(stepSize * 1.0/sqrt(2), stepSize + stepSize * 1.0/sqrt(2))],
                       pens.first!.points)
        XCTAssertEqual([
            float2(0, stepSize),
            float2(stepSize * -1.0/sqrt(2), stepSize + stepSize * 1.0/sqrt(2))],
                       pens[1].points)

    }
}
