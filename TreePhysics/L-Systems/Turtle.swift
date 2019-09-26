import Foundation
import simd

// Basic turtle graphics, inspired by Houdini's L-System implementation:
// https://www.sidefx.com/docs/houdini/nodes/sop/lsystem.html
// Actual drawing happens by means of a `Pen`

enum Command {
    case forward(distance: Float?, width: Float?)
    case tropism(force: float3?)
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

fileprivate let initialOrientation: float3x3 = float3x3(columns: (.y, -.x, .z))

public struct InterpreterConfig {
    public init(
        randomScale: Float = 0,
        randomSeed: Float = 0,
        angle: Float = .pi/8,
        thickness: Float = 0.1,
        thicknessScale: Float = 0.9,
        stepSize: Float = 0.1,
        stepSizeScale: Float = 0.9,
        tropism: float3 = float3.zero,
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
    let tropism: float3
    let elasticity: Float
}

public class Interpreter<P> where P: Pen {
    struct State {
        var position: float3
        var orientation: float3x3 // FIXME make a quat?
        var stepSize: Float // meters
        var thickness: Float // meters^2
        let pen: P

        var heading: float3 {
            return orientation.columns.0
        }
    }

    let configuration: InterpreterConfig
    var stack: [State] = []

    static func parse(_ s: String) -> [Command] {
        var result: [Command] = []
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
            State(position: float3.zero, orientation: initialOrientation, stepSize: configuration.stepSize, thickness: configuration.thickness, pen: pen))
    }

    public func interpret(_ string: String) {
        interpret(Interpreter.parse(string))
    }

    func interpret(_ commands: [Command]) {
        var state = self.stack.removeLast()
        state.pen.start(at: state.position, thickness: state.thickness)

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
                state.position += state.heading * distanceScaled
                state.thickness = thicknessScaled
                _ = state.pen.cont(distance: distanceScaled, tangent: state.heading, thickness: state.thickness)
            case .tropism(_): ()
//                let force = force ?? configuration.force
//                let angle = configuration.elasticity *
//                    length(cross(state.heading, force))
//                state.heading = rotate(state.heading, by: angle, axis: state.up)
            case let .turnLeft(radians):
                let radians = abs(radians ?? configuration.angle)
                state.orientation *= matrix3x3_rotation_up(radians: radians)
            case let .turnRight(radians):
                let radians = -abs(radians ?? configuration.angle)
                state.orientation *= matrix3x3_rotation_up(radians: radians)
            case let .turnRandom(upToRadians):
                _ = upToRadians ?? configuration.angle
//                state.heading = rotate(state.heading, by: Float.random(in: -upTo...upTo), axis: state.up)
            case let .rollRight(radians):
                let radians = -abs(radians ?? configuration.angle)
                state.orientation *= matrix3x3_rotation_heading(radians: radians)
            case let .pitchDown(radians):
                let radians = abs(radians ?? configuration.angle)
                state.orientation *= matrix3x3_rotation_left(radians: radians)
            case let .multiplyStepSize(stepSizeScale):
                state.stepSize *= stepSizeScale ?? configuration.stepSizeScale
            case let .multiplyThickness(thicknessScale):
                state.thickness *= thicknessScale ?? configuration.thicknessScale
            case let .copy(scale):
                let scale = scale ?? state.stepSize
                _ = state.pen.copy(scale: scale, orientation: simd_quatf(state.orientation))
            case .push:
                self.stack.append(state)
                let pen = state.pen.branch
                pen.start(at: state.position, thickness: state.thickness)
                state = State(position: state.position, orientation: state.orientation, stepSize: state.stepSize, thickness: state.thickness, pen: pen)
            case .pop:
                state = self.stack.removeLast()
            }
        }
        stack.append(state)
    }
}

// cf http://algorithmicbotany.org/papers/hanan.dis1992.pdf

fileprivate func matrix3x3_rotation_up(radians: Float) -> float3x3 {
    let cs = cosf(radians)
    let sn = sinf(radians)
    return float3x3(columns:
        (float3(cs, -sn, 0),
         float3(sn, cs, 0),
         float3(0, 0, 1)))
}

fileprivate func matrix3x3_rotation_left(radians: Float) -> float3x3 {
    let cs = cosf(radians)
    let sn = sinf(radians)
    return float3x3(columns:
        (float3(cs, 0, sn),
         float3(0, 1, 0),
         float3(-sn, 0, cs)))
}

fileprivate func matrix3x3_rotation_heading(radians: Float) -> float3x3 {
    let cs = cosf(radians)
    let sn = sinf(radians)
    return float3x3(columns:
        (float3(1, 0, 0),
         float3(0, cs, sn),
         float3(0, -sn, cs)))
}
