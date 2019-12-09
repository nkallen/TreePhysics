import Foundation
import PathKit
import Commander

extension Path {
    public var isMetalSourceFile: Bool {
        return !self.isDirectory && self.extension == "metal"
    }

    public var isMetalLibFile: Bool {
        return !self.isDirectory && self.extension == "metallib"
    }
}

extension Path: ArgumentConvertible {
    /// :nodoc:
    public init(parser: ArgumentParser) throws {
        if let path = parser.shift() {
            self.init(path)
        } else {
            throw ArgumentError.missingValue(argument: nil)
        }
    }
}
