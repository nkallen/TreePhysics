import Foundation
import simd

protocol Pen {
    func start(at: float2, tangent: float2)
    func cont(at: float2, tangent: float2)
    func end(at: float2, tangent: float2)
}

enum Command {
    case forward(distance: Float?, width: Float?)
    case tropism(magnitude: Float?)
    case turnLeft(radians: Float?)
    case turnRight(radians: Float?)
    case turnRandom(upToRadians: Float?)
    case multiplyStepSize(Float?)
    case multiplyThickness(Float?)
    case push
    case pop
}

struct State {
    var position: float2
    var heading: float2
    var stepSize: Float // meters
    var thickness: Float // meters^2
    let pen: Pen
}

struct Configuration {
    let randomScale: Float
    let randomSeed: Float
    let angle: Float
    let thickness: Float
    let thicknessScale: Float
    let stepSize: Float
    let stepSizeScale: Float
    let tropism: Float
    let elasticity: Float
}

class Interpreter {
    static let defaultConfiguration = Configuration(
        randomScale: 0,
        randomSeed: 0,
        angle: .pi/4,
        thickness: 0.1,
        thicknessScale: 0.9,
        stepSize: 0.1,
        stepSizeScale: 0.9,
        tropism: 0,
        elasticity: 0.1)
    let configuration: Configuration
    let penFactory: () -> Pen
    var stack: [State] = []

    init(configuration: Configuration = Interpreter.defaultConfiguration, _ penFactory: @escaping () -> Pen) {
        self.configuration = configuration
        self.penFactory = penFactory
        stack.append(
            State(position: float2.zero, heading: float2(0,1), stepSize: configuration.stepSize, thickness: configuration.thickness, pen: penFactory()))
    }

    func interpret(commands: [Command]) {
        var state = self.stack.removeLast()
        state.pen.start(at: state.position, tangent: state.heading)

        for command in commands {
            switch command {
            case let .forward(distance, thickness):
                state.position += state.heading * (distance ?? state.stepSize)
                state.thickness = thickness ?? state.stepSize
                state.pen.cont(at: state.position, tangent: state.heading)
            case let .tropism(magnitude):
                let angle = configuration.elasticity *
                    length(cross(state.heading, float2(0, -(magnitude ?? configuration.tropism))))
                state.heading = rotate(state.heading, by: angle)
            case let .turnLeft(radians):
                state.heading = rotate(state.heading, by: -abs(radians ?? configuration.angle))
            case let .turnRight(radians):
                state.heading = rotate(state.heading, by: abs(radians ?? configuration.angle))
            case let .turnRandom(upToRadians):
                let upTo = upToRadians ?? configuration.angle
                state.heading = rotate(state.heading, by: Float.random(in: -upTo...upTo))
            case let .multiplyStepSize(stepSizeScale):
                state.stepSize *= stepSizeScale ?? configuration.stepSizeScale
            case let .multiplyThickness(thicknessScale):
                state.thickness *= thicknessScale ?? configuration.thicknessScale
            case .push:
                self.stack.append(state)
                let pen = penFactory()
                pen.start(at: state.position, tangent: state.heading)
                state = State(position: state.position, heading: state.heading, stepSize: state.stepSize, thickness: state.thickness, pen: pen)
            case .pop:
                state = self.stack.removeLast()
            }
        }
        stack.append(state)
    }
}
