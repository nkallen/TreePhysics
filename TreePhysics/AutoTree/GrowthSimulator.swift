import Foundation
import simd

struct AutoTreeConfig {
    let branchingAngle: Float = .pi/4
    let phyllotacticAngle: Float = .pi/4
    let internodeLength: Float = 0.05

    let occupationRadius: Float = 0.05 * 0.6
    let perceptionAngle: Float = .pi/4
    let perceptionRadius: Float = 0.05 * 12

    let n: Float = 1.1 // FIXME rename
    let extremityRadius: Float = 0.001
    let baseRadius: Float = 0.05

    let fullExposure: Float = 10
    let shadowIntensity: Float = 1
}

extension AutoTree {
    final class GrowthSimulator {
        let config: AutoTreeConfig
        var attractionPoints: Set<float3> = []
        let hash: LocalitySensitiveHash<Bud>
        let shadowGrid: ShadowGrid
        var buds: Set<Bud> = []

        init(_ config: AutoTreeConfig) {
            self.config = config
            self.hash = LocalitySensitiveHash<Bud>(cellSize: config.occupationRadius + config.perceptionRadius)
            self.shadowGrid = ShadowGrid(ShadowGridConfig(cellSize: config.internodeLength))
        }

        func add(_ node: Node) {
            switch node {
            case let bud as Bud:
                hash.add(bud)
                buds.insert(bud)
                shadowGrid[node.position] += config.shadowIntensity
            case is Internode:
                shadowGrid[node.position] += config.shadowIntensity
            default: ()
            }

            for child in node.children {
                add(child)
            }
        }

        internal func updateVigor() {
            let start = Date()
            for bud in buds {
                let lightExposure = self.lightExposure(of: bud)
                if let internode = bud.parent as? Internode {
                    internode.lightExposure += lightExposure
                }
            }
            print("finishsed in ", Date().timeIntervalSince(start))
        }

        func update() {
            updateVigor()

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
                let (internode, buds) = bud.grow(towards: Array(points))
                hash.remove(bud)
                self.buds.remove(bud)
                shadowGrid[bud.position] -= config.shadowIntensity
                shadowGrid[internode.position] += config.shadowIntensity

                for bud in buds {
                    hash.add(bud)
                    self.buds.insert(bud)
                    shadowGrid[bud.position] += config.shadowIntensity
                }
            }
        }

        func lightExposure(of bud: Bud) -> Float {
            return max(config.fullExposure - shadowGrid[bud.position] + config.shadowIntensity, 0)
        }
    }
}
