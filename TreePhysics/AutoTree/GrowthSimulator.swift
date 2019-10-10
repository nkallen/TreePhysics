import Foundation
import simd

extension AutoTree {
    public final class GrowthSimulator {
        let config: Config

        var generation = 0
        var attractionPoints: Set<SIMD3<Float>> = []
        let hash: LocalitySensitiveHash<Bud>
        let shadowGrid: ShadowGrid // FIXME all wrong
        var root: Parent!
        var buds: Set<Bud> = []

        init(_ config: Config, shadowGrid: ShadowGrid) {
            self.config = config
            self.hash = LocalitySensitiveHash<Bud>(cellSize: config.occupationRadius + config.perceptionRadius)
            self.shadowGrid = shadowGrid
        }

        public func add(_ parent: Parent) {
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

        internal func updateLightExposure() -> [Node:Float] {
            var lightExposures: [Node:Float] = [:]
            func lightExposure(of bud: Bud) -> Float {
                return max(config.fullExposure - shadowGrid[bud.position] + config.shadowIntensity, 0)
            }

            for bud in buds {
                let l = lightExposure(of: bud)
                lightExposures[bud] = l
                var node: Node = bud
                while let internode = node.parent {
                    lightExposures[internode, default: 0] += l
                    node = internode
                }
            }
            return lightExposures
        }

        internal func updateVigor(exposures: [Node:Float]) -> [Node:Float] {
            var vigors: [Node:Float] = [:]

            func unbiasedVigor(for node: Node) -> Float {
                let l = exposures[node]!
                return pow(l, config.sensitivityOfBudsToLight)
            }

            func recurse(_ parent: Parent) {
                let baseVigor = vigors[parent]!

                if let main = parent.mainChild, let lateral = parent.lateralChild {
                    if baseVigor == 0 {
                        vigors[main] = 0
                        vigors[lateral] = 0
                    } else {
                        let unbiasedMainVigor = unbiasedVigor(for: main)
                        let unbiasedLateralVigor = unbiasedVigor(for: lateral)

                        let denominator: Float = (config.biasVigorTowardsMainAxis * unbiasedMainVigor + (1 - config.biasVigorTowardsMainAxis) * unbiasedLateralVigor) / baseVigor
                        vigors[main] = config.biasVigorTowardsMainAxis * unbiasedMainVigor / denominator
                        vigors[lateral] = (1 - config.biasVigorTowardsMainAxis) * unbiasedLateralVigor / denominator
                    }
                    if let internode = main    as? Internode { recurse(internode) }
                    if let internode = lateral as? Internode { recurse(internode) }
                } else if let main = parent.mainChild {
                    vigors[main] = baseVigor
                    if let internode = main    as? Internode { recurse(internode) }
                } else {
                    fatalError()
                }
            }

            vigors[root] = unbiasedVigor(for: root)
            recurse(root)

            return vigors
        }

        internal func selectBudsWithSpace() -> [Bud:Set<SIMD3<Float>>] {
            var selectedBuds: [Bud:Set<SIMD3<Float>>] = [:]
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
            return selectedBuds
        }

        internal func growShoots(selectedBuds: [Bud:Set<SIMD3<Float>>], vigors: [Node:Float]) {
            guard selectedBuds.count > 0 else { return }

            var maxVigor: Float = 0
            for (bud, _) in selectedBuds {
                maxVigor = max(maxVigor, vigors[bud]!)
            }
            assert(maxVigor > 0)

            for (bud, points) in selectedBuds {
                let shootLength = Int(Float(config.maxShootLength) * vigors[bud]! / maxVigor)
                var currentBud = bud
                for _ in 0..<shootLength {
                    let (internode, (terminalBud, lateralBud)) = currentBud.grow(towards: Array(points))
                    hash.remove(currentBud)
                    self.buds.remove(currentBud)
                    shadowGrid[currentBud.position] -= config.shadowIntensity
                    shadowGrid[internode.position] += config.shadowIntensity
                    internode.generation = generation

                    for bud in [terminalBud, lateralBud].compactMap({ $0 }) {
                        bud.generation = generation
                        hash.add(bud)
                        self.buds.insert(bud)
                        shadowGrid[bud.position] += config.shadowIntensity
                    }
                    currentBud = terminalBud
                }
            }

        }

        public func update() -> Bool {
            guard attractionPoints.count > 0 else { return false }

            let selectedBuds = selectBudsWithSpace()
            guard selectedBuds.count > 0 else { return false }

            let exposures = updateLightExposure()
            let vigors = updateVigor(exposures: exposures)
            growShoots(selectedBuds: selectedBuds, vigors: vigors)
            generation += 1
            return true
        }
    }
}
