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
    let k: Float = 1.1 // FIXME rename bud sensitivy to light >= 0
    let lambda: Float = 0.5 // 0..1
}

extension AutoTree {
    final class GrowthSimulator {
        let config: AutoTreeConfig
        var attractionPoints: Set<float3> = []
        let hash: LocalitySensitiveHash<Bud>
        let shadowGrid: ShadowGrid
        var root: Parent!
        var buds: Set<Bud> = []

        init(_ config: AutoTreeConfig, shadowGrid: ShadowGrid) {
            self.config = config
            self.hash = LocalitySensitiveHash<Bud>(cellSize: config.occupationRadius + config.perceptionRadius)
            self.shadowGrid = shadowGrid
        }

        func add(_ parent: Parent) {
            func add(_ node: Node) {
                shadowGrid[node.position] += config.shadowIntensity

                switch node {
                case let bud as Bud:
                    hash.add(bud)
                    buds.insert(bud)
                case let parent as Parent:
                    if let mainChild = parent.mainChild {
                        add(mainChild)
                    }
                    if let lateralChild = parent.lateralChild {
                        add(lateralChild)
                    }
                default: ()
                }
            }

            self.root = parent
            add(parent as Node)
        }

        internal func updateLightExposure() -> [Parent:Float] {
            var lightExposures: [Parent:Float] = [:]
            func lightExposure(of bud: Bud) -> Float {
                return max(config.fullExposure - shadowGrid[bud.position] + config.shadowIntensity, 0)
            }

            for bud in buds {
                let l = lightExposure(of: bud)
                var node: Node = bud
                while let internode = node.parent {
                    lightExposures[internode] = lightExposures[internode, default: 0] + l
                    node = internode
                }
            }
            return lightExposures
        }

        func updateVigor(exposures: [Parent:Float]) -> [Parent:Float] {
            var vigors: [Parent:Float] = [:]

            func unbiasedVigor(for parent: Parent) -> Float {
                let l = exposures[parent]!
                return pow(l, config.k)
            }

            func recurse(_ parent: Parent) {
                let baseVigor = vigors[parent]!

                if let main = parent.mainChild as? Internode, let lateral = parent.lateralChild as? Internode {
                    let unbiasedMainVigor = unbiasedVigor(for: main)
                    let unbiasedLateralVigor = unbiasedVigor(for: lateral)

                    let denominator: Float = (config.lambda * unbiasedMainVigor + (1 - config.lambda) * unbiasedLateralVigor) / baseVigor
                    print("denominator", denominator)
                    vigors[main] = config.lambda * unbiasedMainVigor / denominator
                    vigors[lateral] = (1 - config.lambda) * unbiasedLateralVigor / denominator
                    recurse(main)
                    recurse(lateral)
                } else if let node = (parent.mainChild ?? parent.lateralChild) as? Internode {
                    vigors[node] = baseVigor
                    recurse(node)
                }
            }

            vigors[root] = unbiasedVigor(for: root)
            recurse(root)

            return vigors
        }

        func update() {
            let exposures = updateLightExposure()
            _ = updateVigor(exposures: exposures)

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
                let (internode, (lateralBud, terminalBud)) = bud.grow(towards: Array(points))
                hash.remove(bud)
                self.buds.remove(bud)
                shadowGrid[bud.position] -= config.shadowIntensity
                shadowGrid[internode.position] += config.shadowIntensity

                for bud in [lateralBud, terminalBud].compactMap({ $0 }) {
                    hash.add(bud)
                    self.buds.insert(bud)
                    shadowGrid[bud.position] += config.shadowIntensity
                }
            }
        }
    }
}
