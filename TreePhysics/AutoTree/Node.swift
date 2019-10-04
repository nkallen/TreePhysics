import Foundation
import simd

fileprivate let branchingAngle: Float = .pi/4
fileprivate let phyllotacticAngle: Float = .pi/4
fileprivate let length: Float = 1
fileprivate let radius: Float = 1

fileprivate let occupationRadius: Float = 1
fileprivate let perceptionAngle: Float = 1
fileprivate let perceptionRadius: Float = 1

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
        func grow(towards points: [float3]) -> (Internode, [Bud]) {
            guard let parent = parent else { fatalError() }

            var newDirection = float3.zero
            for point in points {
                let directionToAttractionPoint = point - self.position
                newDirection += normalize(directionToAttractionPoint)
            }

            let newOrientation = simd_quatf(from: orientation.heading, to: normalize(newDirection))
            let branchingRotation = simd_quatf(angle: branchingAngle, axis: newOrientation.up)
            let phyllotacticRotation = simd_quatf(angle: phyllotacticAngle, axis: newOrientation.heading)

            let lateralBud  = Bud(position: position, orientation: branchingRotation * newOrientation)
            parent.addChild(lateralBud)
            let internode = Internode(position: position, orientation: newOrientation, length: length, radius: radius)
            parent.addChild(internode)
            let terminalBud = Bud(position: position + internode.length, orientation: phyllotacticRotation * newOrientation)
            internode.addChild(terminalBud)

            let buds = [lateralBud, terminalBud]

            self.removeFromParent()

            return (internode, buds)
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
                print(point)
                var closestBud: Bud?
                var closestDistance = Float.infinity
                let nearbyBuds = hash.elements(near: point)
                for bud in nearbyBuds {
                    if bud.occupies(point: point) {
                        attractionPoints.remove(point)
                        closestBud = nil
                        break
                    } else if bud.perceives(point: point) {
                        var attractionPointsForBud = selectedBuds[bud] ?? []
                        attractionPointsForBud.insert(point)
                        selectedBuds[bud] = attractionPointsForBud
                        let dist = distance(bud.position, point)
                        if dist < closestDistance {
                            closestBud = bud
                            closestDistance = dist
                        }
                    }
                }
                if let closestBud = closestBud {
                    let attractionPointsForBud = selectedBuds[closestBud]!

                    let (_, buds) = closestBud.grow(towards: Array(attractionPointsForBud))
                    hash.remove(closestBud)
                    for bud in buds { hash.add(bud) }
                }
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
