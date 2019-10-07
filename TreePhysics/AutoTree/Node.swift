import Foundation
import simd

extension AutoTree {
    class Node {
        let config: AutoTreeConfig

        weak var parent: Node? = nil
        var children: Set<Node> = []
        let position: float3
        let orientation: simd_quatf

        init(config: AutoTreeConfig, position: float3, orientation: simd_quatf) {
            self.config = config
            self.position = position
            self.orientation = orientation
        }

        func removeFromParent() {
            parent?.children.remove(self)
            parent = nil
        }

        func addChild(_ node: Node) {
            children.insert(node)
            node.parent = self
        }
    }

    class Bud: Node {
        fileprivate override init(config: AutoTreeConfig, position: float3, orientation: simd_quatf) {
            super.init(config: config, position: position, orientation: orientation)
        }

        func grow(towards points: [float3], produceLateralBud: Bool) -> (Internode, [Bud]) {
            guard let parent = parent else { fatalError("\(self) has no parent") }

            var newDirection = float3.zero
            for point in points {
                let directionToAttractionPoint = point - self.position
                newDirection += normalize(directionToAttractionPoint)
            }
            newDirection = normalize(newDirection)

            let newOrientation = (simd_quatf(from: orientation.heading, to: newDirection) * orientation).normalized
            let branchingRotation = simd_quatf(angle: config.branchingAngle, axis: newOrientation.up)
            let phyllotacticRotation = simd_quatf(angle: config.phyllotacticAngle, axis: newOrientation.heading)

            var buds: [Bud] = []

            if produceLateralBud {
                let lateralBud = LateralBud(config: config, position: position, orientation: (branchingRotation * newOrientation).normalized)
                parent.addChild(lateralBud)
                buds.append(lateralBud)
            }

            let internode = Internode(config: config, position: position, orientation: newOrientation, length: config.length, radius: config.radius)
            parent.addChild(internode)

            let terminalBud = TerminalBud(config: config, position: position + internode.length * newOrientation.heading, orientation: (phyllotacticRotation * newOrientation).normalized)
            buds.append(terminalBud)
            internode.addChild(terminalBud)

            self.removeFromParent()

            return (internode, buds)
        }

        func grow(towards points: [float3]) -> (AutoTree.Internode, [AutoTree.Bud]) {
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
    }

    final class TerminalBud: Bud {
        public override init(config: AutoTreeConfig, position: float3, orientation: simd_quatf) {
            super.init(config: config, position: position, orientation: orientation)
        }

        override func grow(towards points: [float3]) -> (AutoTree.Internode, [AutoTree.Bud]) {
            return grow(towards: points, produceLateralBud: parent is Internode)
        }
    }

    final class LateralBud: Bud {
        public override init(config: AutoTreeConfig, position: float3, orientation: simd_quatf) {
            super.init(config: config, position: position, orientation: orientation)
        }

        override func grow(towards points: [float3]) -> (AutoTree.Internode, [AutoTree.Bud]) {
            return grow(towards: points, produceLateralBud: false)
        }
    }

    class Internode: Node {
        let length: Float
        let radius: Float

        init(config: AutoTreeConfig, position: float3, orientation: simd_quatf, length: Float, radius: Float) {
            self.length = length
            self.radius = radius
            super.init(config: config, position: position, orientation: orientation)
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
