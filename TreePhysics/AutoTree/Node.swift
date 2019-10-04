import Foundation
import simd

fileprivate let branchingAngle: Float = .pi/4
fileprivate let phyllotacticAngle: Float = .pi/4
fileprivate let length: Float = 0.05
fileprivate let radius: Float = 0.01

fileprivate let occupationRadius: Float = length
fileprivate let perceptionAngle: Float = .pi/4
fileprivate let perceptionRadius: Float = 0.2

extension AutoTree.Node: HasPosition {}

extension AutoTree.Node: Equatable, Hashable {
    static func == (lhs: AutoTree.Node, rhs: AutoTree.Node) -> Bool {
        return lhs === rhs
    }

    func hash(into hasher: inout Hasher) {
        hasher.combine(ObjectIdentifier(self))
    }
}

extension AutoTree {
    static func root() -> Node {
        return Node(position: .zero, orientation: .identity)
    }

    class Node {
        weak var parent: Node? = nil
        var children: Set<Node> = []
        let position: float3
        let orientation: simd_quatf

        init(position: float3, orientation: simd_quatf) {
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
        func grow(towards points: [float3], produceLateralBud: Bool) -> (Internode, [Bud]) {
            guard let parent = parent else { fatalError("\(self) has no parent") }

            var newDirection = float3.zero
            for point in points {
                let directionToAttractionPoint = point - self.position
                newDirection += normalize(directionToAttractionPoint)
            }
            newDirection = normalize(newDirection)

            let newOrientation = simd_quatf(from: orientation.heading, to: newDirection)
            let branchingRotation = simd_quatf(angle: branchingAngle, axis: newOrientation.up)
            let phyllotacticRotation = simd_quatf(angle: phyllotacticAngle, axis: newOrientation.heading)

            var buds: [Bud] = []

            if produceLateralBud {
                let lateralBud = LateralBud(position: position, orientation: branchingRotation * newOrientation)
                parent.addChild(lateralBud)
                buds.append(lateralBud)
            }

            let internode = Internode(position: position, orientation: newOrientation, length: length, radius: radius)
            parent.addChild(internode)

            let terminalBud = TerminalBud(position: position + internode.length * newOrientation.heading, orientation: phyllotacticRotation * newOrientation)
            buds.append(terminalBud)
            internode.addChild(terminalBud)

            self.removeFromParent()

            return (internode, buds)
        }

        func grow(towards points: [float3]) -> (AutoTree.Internode, [AutoTree.Bud]) {
            fatalError("Abstract method")
        }

        func occupies(point: float3) -> Bool {
            return distance(position, point) < occupationRadius
        }

        func perceives(point: float3) -> Bool {
            let direction = point - position
            let dist = simd_length(direction)
            if dist > perceptionRadius + occupationRadius { return false }
            return simd_quatf(from: orientation.heading, to: direction).angle <= perceptionAngle
        }
    }

    final class TerminalBud: Bud {
        override func grow(towards points: [float3]) -> (AutoTree.Internode, [AutoTree.Bud]) {
            return grow(towards: points, produceLateralBud: parent is Internode)
        }
    }

    final class LateralBud: Bud {
        override func grow(towards points: [float3]) -> (AutoTree.Internode, [AutoTree.Bud]) {
            return grow(towards: points, produceLateralBud: false)
        }
    }

    class Internode: Node {
        let length: Float
        let radius: Float

        init(position: float3, orientation: simd_quatf, length: Float, radius: Float) {
            self.length = length
            self.radius = radius
            super.init(position: position, orientation: orientation)
        }
    }

    class GrowthSimulator {
        var attractionPoints: Set<float3> = []
        let hash = LocalitySensitiveHash<Bud>(cellSize: perceptionRadius)

        func update() {
            var selectedBuds: [Bud:Set<float3>] = [:]

            for point in attractionPoints {
                var closestBud: Bud?
                var closestDistance = Float.infinity
                let nearbyBuds = hash.elements(near: point)
                for bud in nearbyBuds {
                    if bud.occupies(point: point) {
                        attractionPoints.remove(point)
                        print("removing attraction point \(point); now \(attractionPoints.count)")
                        closestBud = nil
                        break
                    } else if bud.perceives(point: point) {
                        let dist = distance(bud.position, point)
                        if dist < closestDistance {
                            closestBud = bud
                            closestDistance = dist
                        }
                    } else {}
                }
                if let closestBud = closestBud {
                    var attractionPointsForBud = selectedBuds[closestBud] ?? []
                    attractionPointsForBud.insert(point)
                    selectedBuds[closestBud] = attractionPointsForBud
                }
            }
            for (bud, points) in selectedBuds {
                let (_, buds) = bud.grow(towards: Array(points))
                hash.remove(bud)
                for bud in buds { hash.add(bud) }
            }
        }
    }
}

extension simd_quatf {
    var left: float3 {
        return act(.x)
    }

    var up: float3 {
        return act(.z)
    }

    var heading: float3 {
        return act(.y)
    }
}
