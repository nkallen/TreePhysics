import Foundation

struct check {
    static let angle = 0...(2 * Float.pi)
    static let positive = 0...

    static func angle(_ x: Float) {
        precondition((0...(2 * Float.pi)).contains(x))
    }

    static func positive(_ x: Float) {
        precondition(x > 0)
    }

    static func positive(_ x: Int) {
        precondition(x > 0)
    }

    static func nonneg(_ x: Float) {
        precondition(x >= 0)
    }

    static func normal(_ x: Float) {
        precondition((0...1).contains(x))
    }

    static func assert(_ x: Bool) {
        precondition(x)
    }
}

public func assert(_ condition: @autoclosure () -> Bool, _ message: @autoclosure () -> String = String(), file: StaticString = #file, line: UInt = #line, otherwise: (String) -> ()) {
    if !condition() {
        print("Assertion Failure: \(message()) at \(file):\(line)")
        otherwise(message())
    }
}
