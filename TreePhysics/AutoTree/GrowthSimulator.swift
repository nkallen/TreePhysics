import Foundation
import simd

struct AutoTreeConfig {
    let branchingAngle: Float = .pi/4
    let phyllotacticAngle: Float = .pi/4
    let internodeLength: Float = 0.05
    let radius: Float = 0.01

    let occupationRadius: Float = 0.05
    let perceptionAngle: Float = .pi/4
    let perceptionRadius: Float = 1

    let n: Float = 1.2
    let extremityRadius: Float = 0.001
    let baseRadius: Float = 0.05
}

extension AutoTree {
    class GrowthSimulator {
        let config: AutoTreeConfig
        var attractionPoints: Set<float3> = []
        let hash: LocalitySensitiveHash<Bud>

        init(_ config: AutoTreeConfig) {
            self.config = config
            self.hash = LocalitySensitiveHash<Bud>(cellSize: config.perceptionRadius)
        }

        func add(_ node: Node) {
            switch node {
            case let bud as Bud:
                hash.add(bud)
            default:
                for child in node.children {
                    add(child)
                }
            }
        }

        func update() {
            var selectedBuds: [Bud:Set<float3>] = [:]

            for point in attractionPoints {
                var closestBud: Bud?
                var closestDistance = Float.infinity
                let nearbyBuds = hash.elements(near: point)
                for bud in nearbyBuds {
                    if bud.occupies(point: point) {
                        attractionPoints.remove(point)
                        closestBud = nil
                        break
                    } else if bud.perceives(point: point) {
                        let dist = distance(bud.position, point)
                        if dist < closestDistance {
                            closestBud = bud
                            closestDistance = dist
                        }
                    }
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
