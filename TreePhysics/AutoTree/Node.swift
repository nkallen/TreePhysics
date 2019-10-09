import Foundation
import simd

extension AutoTree {
    class Node {
        let config: AutoTreeConfig

        weak var parent: Node? = nil
        private(set) var lateralChild: Node? = nil
        private(set) var mainChild: Node? = nil

        let position: float3
        let orientation: simd_quatf

        private(set) var terminalBranchCount: Int = 0 {
            didSet {
                let delta = terminalBranchCount - oldValue
                parent?.terminalBranchCount += delta
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
                    if internode.terminalBranchCount < thickest_.terminalBranchCount {
                        thickest = internode
                        rest.remove(internode)
                    }
                } else {
                    thickest = internode
                    rest.remove(internode)
                }
            }
            return (thickest, rest)
        }

        init(config: AutoTreeConfig, position: float3, orientation: simd_quatf) {
            self.config = config
            self.position = position
            self.orientation = orientation
        }

        func replaceBud(_ bud: Bud, with internode: Internode) {
            switch bud {
            case let bud as TerminalBud where mainChild == bud:
                mainChild = internode
            case let bud as LateralBud where lateralChild == bud:
                terminalBranchCount += internode.terminalBranchCount
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
                terminalBranchCount += 1
            case let bud as LateralBud where lateralChild == nil:
                self.lateralChild = bud
            default: fatalError()
            }
            bud.parent = self
        }
    }

    class Bud: Node {
        fileprivate override init(config: AutoTreeConfig, position: float3, orientation: simd_quatf) {
            super.init(config: config, position: position, orientation: orientation)
        }

        func grow(towards points: [float3], produceLateralBud: Bool) -> (Internode, (TerminalBud, LateralBud?)) {
            guard let parent = parent else { fatalError("\(self) has no parent") }

            var newDirection = float3.zero
            for point in points {
                let directionToAttractionPoint = point - self.position
                newDirection += normalize(directionToAttractionPoint)
            }
            if newDirection == .zero {
                newDirection = orientation.heading
            }
            newDirection = normalize(newDirection)

            let newOrientation = (simd_quatf(from: orientation.heading, to: newDirection) * orientation).normalized
            let branchingRotation = simd_quatf(angle: config.branchingAngle, axis: newOrientation.up)
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

        func grow(towards points: [float3] = []) -> (Internode, (TerminalBud, LateralBud?)) {
            fatalError("Abstract method")
        }

        func occupies(point: float3) -> Bool {
            return distance(position, point) < config.occupationRadius
        }

        func perceives(point: float3) -> Bool {
            let direction = point - position
            let dist = simd_length(direction)
            if dist > config.perceptionRadius + config.occupationRadius { return false }
            return simd_quatf(from: orientation.heading, to: direction).angle <= config.perceptionAngle
        }

        override func addBud(_ bud: Bud) {
            fatalError("Buds cannot have children")
        }

        override func replaceBud(_ bud: Bud, with internode: Internode) {
            fatalError("Buds cannot have children")
        }
    }

    final class TerminalBud: Bud {
        public override init(config: AutoTreeConfig, position: float3, orientation: simd_quatf) {
            super.init(config: config, position: position, orientation: orientation)
        }

        override func grow(towards points: [float3] = []) -> (Internode, (TerminalBud, LateralBud?)) {
            return grow(towards: points, produceLateralBud: parent is Internode)
        }
    }

    final class LateralBud: Bud {
        public override init(config: AutoTreeConfig, position: float3, orientation: simd_quatf) {
            super.init(config: config, position: position, orientation: orientation)
        }

        override func grow(towards points: [float3] = []) -> (Internode, (TerminalBud, LateralBud?)) {
            return grow(towards: points, produceLateralBud: false)
        }
    }

    final class Internode: Node {
        func diameter(exponent: Float) -> Float {
            return pow(Float(terminalBranchCount) * pow(2*config.extremityRadius, exponent), 1/exponent)
        }
    }
}

extension AutoTree.Node: HasPosition {}

extension AutoTree.Node: Equatable, Hashable {
    static func == (lhs: AutoTree.Node, rhs: AutoTree.Node) -> Bool {
        return lhs === rhs
    }

    func hash(into hasher: inout Hasher) {
        hasher.combine(ObjectIdentifier(self))
    }
}
