import Foundation
import simd

fileprivate var i = 0

extension AutoTree {
    public class Node {
        let name: String
        let config: Config
        var generation: Int?

        weak var parent: Parent? = nil

        let position: SIMD3<Float>
        let orientation: simd_quatf

        fileprivate init(config: Config, position: SIMD3<Float>, orientation: simd_quatf) {
            self.config = config
            self.position = position
            self.orientation = orientation
            self.name = "\(type(of: self))[\(i)]"
            i += 1
        }

    }

    public class Parent: Node {
        private(set) var lateralChild: Node? = nil
        private(set) var mainChild: Node? = nil

        private(set) var terminalBudCount: Int = 0 {
            didSet {
                let delta = terminalBudCount - oldValue
                parent?.terminalBudCount += delta
            }
        }

        var thickestChild: (Internode?, Set<Node>) {
            var rest: Set<Node> = Set([mainChild, lateralChild].compactMap { $0 })
            var thickest: Internode? = nil
            if case let internode as Internode = lateralChild {
                thickest = internode
                rest.remove(internode)
            }
            if case let internode as Internode = mainChild {
                if let thickest_ = thickest {
                    if internode.terminalBudCount > thickest_.terminalBudCount {
                        thickest = internode
                        rest.remove(internode)
                        rest.insert(thickest_)
                    }
                } else {
                    thickest = internode
                    rest.remove(internode)
                }
            }
            return (thickest, rest)
        }

        public override init(config: Config, position: SIMD3<Float>, orientation: simd_quatf) {
            super.init(config: config, position: position, orientation: orientation)
        }

        func replaceBud(_ bud: Bud, with internode: Internode) {
            switch bud {
            case let bud as TerminalBud where mainChild == bud:
                mainChild = internode
            case let bud as LateralBud where lateralChild == bud:
                terminalBudCount += internode.terminalBudCount
                lateralChild = internode
            default: fatalError()
            }
            internode.parent = self
            bud.parent = nil
        }

        func addBud(_ bud: Bud) {
            switch bud {
            case let bud as TerminalBud where mainChild == nil:
                self.mainChild = bud
                terminalBudCount += 1
            case let bud as LateralBud where lateralChild == nil:
                self.lateralChild = bud
            default: fatalError()
            }
            bud.parent = self
        }
    }

    public class Bud: Node {
        func growthDirection(towards points: [SIMD3<Float>]) -> SIMD3<Float> {
            let gravitropismDirection: SIMD3<Float> = simd_quatf(angle: config.gravitropismAngle, axis:  cross(.y, orientation.heading)).act(.y)

            var newDirection: SIMD3<Float>

            var environmentalDirection: SIMD3<Float> = .zero
            if points.count > 0 {
                for point in points {
                    let directionToAttractionPoint = point - self.position
                    environmentalDirection += normalize(directionToAttractionPoint)
                }
                environmentalDirection = normalize(environmentalDirection)

                newDirection = config.branchStraightnessBias * orientation.heading
                newDirection += config.branchEnvironmentalBias * environmentalDirection
                newDirection += config.branchGravitropismBias * gravitropismDirection
            } else {
                newDirection = (1 - config.branchGravitropismBias) * orientation.heading
                newDirection += config.branchGravitropismBias * gravitropismDirection
            }
            newDirection = normalize(newDirection)
            assert(newDirection.isFinite)

            return newDirection
        }

        var gravimorphismFactor: Float {
            let parentOrientation: simd_quatf = parent!.orientation
            let theta = parentOrientation.vertical.angle(with:
                orientation.heading.project(ontoPlane: parentOrientation.heading)) + .pi/2
            let tau = parentOrientation.heading.angle(with:  .y)
            let solution = solve_quadratic(
                a: sqr(config.verticalGravimorphismBias*cos(theta)) + sqr(config.horizontalGravimorphismBias*sin(theta)),
                b: -2*sqr(config.horizontalGravimorphismBias)*config.topsideGravimorphismBias*sin(theta),
                c: sqr(config.horizontalGravimorphismBias*config.topsideGravimorphismBias) - sqr(config.horizontalGravimorphismBias*config.verticalGravimorphismBias))
            let b: Float
            switch solution {
            case let .realDistinct(x, y):
                let d = max(x,y)
                b = sqr(cos(tau)) + d*sqr(sin(tau))
            default:
                fatalError("\(solution)")
            }
            return b
        }

        fileprivate func grow(inDirection newDirection: SIMD3<Float>, produceLateralBud: Bool) -> (Internode, (TerminalBud, LateralBud?)) {
            guard let parent = parent else { fatalError("\(self) has no parent") }

            let newOrientation = (simd_quatf(from: orientation.heading, to: newDirection) * orientation).normalized
            let branchingRotation = simd_quatf(angle: -config.branchingAngle, axis: newOrientation.up)
            let phyllotacticRotation = simd_quatf(angle: config.phyllotacticAngle, axis: newOrientation.heading)

            var lateralBud: LateralBud? = nil
            if produceLateralBud {
                lateralBud = LateralBud(config: config, position: position, orientation: (branchingRotation * newOrientation).normalized)
                parent.addBud(lateralBud!)
            }

            let internode = Internode(config: config, position: position, orientation: newOrientation)

            let terminalBud = TerminalBud(config: config, position: position + config.internodeLength * newOrientation.heading, orientation: (phyllotacticRotation * newOrientation).normalized)
            internode.addBud(terminalBud)

            parent.replaceBud(self, with: internode)

            return (internode, (terminalBud, lateralBud))
        }

        func grow(towards points: [SIMD3<Float>] = []) -> (Internode, (TerminalBud, LateralBud?)) {
            let newDirection = growthDirection(towards: points)
            return grow(inDirection: newDirection)
        }

        func grow(inDirection newDirection: SIMD3<Float>) -> (Internode, (TerminalBud, LateralBud?)) {
            fatalError("Abstract method")
        }

        func occupies(point: SIMD3<Float>) -> Bool {
            return distance(position, point) <= config.occupationRadius
        }

        func perceives(point: SIMD3<Float>) -> Bool {
            let direction = point - position
            let dist = simd_length(direction)
            if dist > config.perceptionRadius + config.occupationRadius { return false }
            return acos(dot(orientation.heading, direction)) <= config.perceptionAngle
        }
    }

    public final class TerminalBud: Bud {
        public override init(config: Config, position: SIMD3<Float>, orientation: simd_quatf) {
            super.init(config: config, position: position, orientation: orientation)
        }

        override func grow(inDirection newDirection: SIMD3<Float>) -> (Internode, (TerminalBud, LateralBud?)) {
            return grow(inDirection: newDirection, produceLateralBud: parent is Internode)
        }
    }

    public final class LateralBud: Bud {
        public override init(config: Config, position: SIMD3<Float>, orientation: simd_quatf) {
            super.init(config: config, position: position, orientation: orientation)
        }

        override func grow(inDirection newDirection: SIMD3<Float>) -> (Internode, (TerminalBud, LateralBud?)) {
            return grow(inDirection: newDirection, produceLateralBud: false)
        }
    }

    public final class Internode: Parent {
        func diameter(exponent: Float) -> Float {
            return pow(Float(terminalBudCount) * pow(2*config.extremityRadius, exponent), 1/exponent)
        }
    }
}

extension AutoTree.Node: HasPosition {}

extension AutoTree.Node: Equatable, Hashable {
    public static func == (lhs: AutoTree.Node, rhs: AutoTree.Node) -> Bool {
        return lhs === rhs
    }

    public func hash(into hasher: inout Hasher) {
        hasher.combine(ObjectIdentifier(self))
    }
}
