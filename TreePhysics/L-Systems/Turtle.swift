import Foundation
import simd

// Basic turtle graphics, inspired by Houdini's L-System implementation:
// https://www.sidefx.com/docs/houdini/nodes/sop/lsystem.html
// Actual drawing happens by means of a `Pen`

enum TurtleCommand {
    case forward(distance: Float?, width: Float?)
    case tropism(force: SIMD3<Float>?)
    case turnLeft(radians: Float?)
    case turnRight(radians: Float?)
    case turnRandom(upToRadians: Float?)
    case multiplyStepSize(Float?)
    case multiplyThickness(Float?)
    case rollRight(Float?)
    case pitchDown(Float?)
    case push
    case pop
    case copy(scale: Float?)
}

fileprivate let initialOrientation = simd_quatf.identity

public struct InterpreterConfig {
    public init(
        randomScale: Float = 0,
        randomSeed: Float = 0,
        angle: Float = .pi/8,
        thickness: Float = 0.1,
        thicknessScale: Float = 0.9,
        stepSize: Float = 0.1,
        stepSizeScale: Float = 0.9,
        tropism: SIMD3<Float> = .zero,
        elasticity: Float = 0.1) {
        self.randomScale = randomScale
        self.randomSeed = randomSeed
        self.angle = angle
        self.thickness = thickness
        self.thicknessScale = thicknessScale
        self.stepSize = stepSize
        self.stepSizeScale = stepSizeScale
        self.tropism = tropism
        self.elasticity = elasticity
    }

    let randomScale: Float
    let randomSeed: Float
    let angle: Float
    let thickness: Float
    let thicknessScale: Float
    let stepSize: Float
    let stepSizeScale: Float
    let tropism: SIMD3<Float>
    let elasticity: Float
}

public class Interpreter<P> where P: Pen {
    struct State {
        var position: SIMD3<Float>
        var orientation: simd_quatf
        var stepSize: Float // meters
        var thickness: Float // meters^2
        let pen: P
    }

    let configuration: InterpreterConfig
    var stack: [State] = []

    static func parse(_ s: String) -> [TurtleCommand] {
        var result: [TurtleCommand] = []
        for char in s {
            switch char {
            case "F":
                result.append(.forward(distance: nil, width: nil))
            case "!":
                result.append(.multiplyThickness(nil))
            case "\"":
                result.append(.multiplyStepSize(nil))
            case "-":
                result.append(.turnLeft(radians: nil))
            case "+":
                result.append(.turnRight(radians: nil))
            case "[":
                result.append(.push)
            case "]":
                result.append(.pop)
            case "/":
                result.append(.rollRight(nil))
            case "&":
                result.append(.pitchDown(nil))
            case "J":
                result.append(.copy(scale: nil))
            default: ()
//                fatalError()
            }
        }
        return result
    }

    public init(configuration: InterpreterConfig = InterpreterConfig(), pen: P) {
        self.configuration = configuration
        stack.append(
            State(position: .zero, orientation: initialOrientation, stepSize: configuration.stepSize, thickness: configuration.thickness, pen: pen))
    }

    public func interpret(_ string: String) {
        interpret(Interpreter.parse(string))
    }

    func interpret(_ commands: [TurtleCommand]) {
        var state = self.stack.removeLast()
        state.pen.start(at: state.position, orientation: state.orientation, thickness: state.thickness)

        for command in commands {
            switch command {
            case let .forward(distance, thickness):
                let distance = distance ?? state.stepSize
                let thickness = thickness ?? state.thickness
                let distanceScaled: Float
                let thicknessScaled: Float
                if configuration.randomScale != 0 {
                    let rand = Float.random(in: -configuration.randomScale...configuration.randomScale)
                    distanceScaled = distance + distance * rand
                    thicknessScaled = thickness + thickness * rand
                } else {
                    distanceScaled = distance
                    thicknessScaled = thickness
                }
                state.position += state.orientation.heading * distanceScaled
                state.thickness = thicknessScaled
                _ = state.pen.cont(distance: distanceScaled, orientation: state.orientation, thickness: state.thickness)
            case .tropism(_): ()
//                let force = force ?? configuration.force
//                let angle = configuration.elasticity *
//                    length(cross(state.heading, force))
//                state.heading = rotate(state.heading, by: angle, axis: state.up)
            case let .turnLeft(radians):
                let radians = abs(radians ?? configuration.angle)
                state.orientation = simd_quatf(angle: radians, axis: state.orientation.up) * state.orientation
                state.orientation = state.orientation.normalized
            case let .turnRight(radians):
                let radians = -abs(radians ?? configuration.angle)
                state.orientation = simd_quatf(angle: radians, axis: state.orientation.up) * state.orientation
                state.orientation = state.orientation.normalized
            case let .turnRandom(upToRadians):
                _ = upToRadians ?? configuration.angle
//                state.heading = rotate(state.heading, by: Float.random(in: -upTo...upTo), axis: state.up)
            case let .rollRight(radians):
                let radians = -abs(radians ?? configuration.angle)
                state.orientation = simd_quatf(angle: radians, axis: state.orientation.heading) * state.orientation
                state.orientation = state.orientation.normalized
            case let .pitchDown(radians):
                let radians = abs(radians ?? configuration.angle)
                state.orientation = simd_quatf(angle: radians, axis: state.orientation.left) * state.orientation
                state.orientation = state.orientation.normalized
            case let .multiplyStepSize(stepSizeScale):
                state.stepSize *= stepSizeScale ?? configuration.stepSizeScale
            case let .multiplyThickness(thicknessScale):
                state.thickness *= thicknessScale ?? configuration.thicknessScale
            case let .copy(scale):
                let scale = scale ?? state.stepSize
                _ = state.pen.copy(scale: scale, orientation: state.orientation)
            case .push:
                self.stack.append(state)
                let pen = state.pen.branch()
                pen.start(at: state.position, orientation: state.orientation, thickness: state.thickness)
                state = State(position: state.position, orientation: state.orientation, stepSize: state.stepSize, thickness: state.thickness, pen: pen)
            case .pop:
                state = self.stack.removeLast()
            }
        }
        stack.append(state)
    }
}
