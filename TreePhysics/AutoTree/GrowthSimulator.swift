import Foundation
import simd

fileprivate typealias GlobalError = Error

extension AutoTree {
    public enum Error: GlobalError {
        case noAttractionPoints
        case noSelectedBuds
        case noVigor
    }

    public final class GrowthSimulator {
        let config: Config

        private(set) var generation = 0
        private(set) var attractionPoints: Set<SIMD3<Float>> = []
        private let hash: LocalitySensitiveHash<Bud>
        private let shadowGrid: ShadowGrid
        private var root: Parent!
        private var buds: Set<Bud> = []

        init(_ config: Config, shadowGrid: ShadowGrid) {
            self.config = config
            self.hash = LocalitySensitiveHash<Bud>(cellSize: config.occupationRadius + config.perceptionRadius)
            self.shadowGrid = shadowGrid
        }

        public func addRoot(_ parent: Parent) {
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

        public func addAttractionPoints(_ points: [SIMD3<Float>]) {
            self.attractionPoints.formUnion(points)
        }

        internal func updateLightExposure() -> [Node:Float] {
            var lightExposures: [Node:Float] = [:]

            for bud in buds {
                let l = max(config.fullExposure - shadowGrid[bud.position] + config.shadowIntensity, 0)
                let q = bud.gravimorphismFactor * pow(l, config.sensitivityOfBudsToLight)
                lightExposures[bud] = q
                var node: Node = bud
                while let internode = node.parent {
                    lightExposures[internode, default: 0] += q
                    node = internode
                }
            }
            return lightExposures
        }

        internal func updateVigor(exposures: [Node:Float]) -> [Node:Float] {
            var vigors: [Node:Float] = [:]

            func recurse(_ parent: Parent) {
                let baseVigor = vigors[parent]!

                if let main = parent.mainChild, let lateral = parent.lateralChild {
                    if baseVigor == 0 {
                        vigors[main] = 0
                        vigors[lateral] = 0
                    } else {
                        let qMain = exposures[main]!
                        let qLateral = exposures[lateral]!

                        let denominator: Float = (config.apicalDominance * qMain + (1 - config.apicalDominance) * qLateral) / baseVigor
                        vigors[main] = config.apicalDominance * qMain / denominator
                        vigors[lateral] = (1 - config.apicalDominance) * qLateral / denominator
                    }
                    if let internode = main    as? Internode { recurse(internode) }
                    if let internode = lateral as? Internode { recurse(internode) }
                } else if let main = parent.mainChild {
                    vigors[main] = baseVigor
                    if let internode = main as? Internode { recurse(internode) }
                } else {
                    fatalError()
                }
            }

            vigors[root] = exposures[root]!
            recurse(root)

            return vigors
        }

        internal func selectBudsWithSpace() -> [Bud:Set<SIMD3<Float>>] {
            var selectedBuds: [Bud:Set<SIMD3<Float>>] = [:]
            for point in attractionPoints {
                var closestBud: Bud?
                var closestDistance = Float.infinity
                let nearbyBuds = hash.elements(near: point)
                var occupies = 0, perceives = 0
                for bud in nearbyBuds {
                    if bud.occupies(point: point) {
                        attractionPoints.remove(point)
                        closestBud = nil
                        occupies += 1
                        break
                    } else if bud.perceives(point: point) {
                        let dist = distance(bud.position, point)
                        if dist < closestDistance {
                            closestBud = bud
                            closestDistance = dist
                        }
                        perceives += 1
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

        internal func growShoots(selectedBuds: [Bud:Set<SIMD3<Float>>], vigors: [Node:Float], maxVigor: Float) {
            guard selectedBuds.count > 0 else { return }

            for (bud, points) in selectedBuds {
                let shootLength = Int(floor(Float(config.maxShootLength) * vigors[bud]! / maxVigor))
                let newDirection = bud.growthDirection(towards: Array(points))
                var currentBud = bud
                for _ in 0..<shootLength {
                    let (internode, (terminalBud, lateralBud)) = currentBud.grow(inDirection: newDirection)
                    hash.remove(currentBud)
                    self.buds.remove(currentBud)
                    // NOTE: For now, the internode and bud it replaces are at the same location, so changing the shadow grid is a no-op
//                    shadowGrid[currentBud.position] -= config.shadowIntensity
//                    shadowGrid[internode.position] += config.shadowIntensity
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

        public func update(enableAllBuds: Bool = false) throws -> [Bud] {
            var terminalBudCount = 0
            var lateralBudCount = 0
            for bud in buds {
                if bud is TerminalBud {
                    terminalBudCount += 1
                } else if bud is LateralBud {
                    lateralBudCount += 1
                }
            }
            if enableAllBuds {
                for bud in buds {
                    let point = bud.position + bud.orientation.heading * (config.occupationRadius + config.perceptionRadius)
                    self.attractionPoints.insert(point)
                }
            }
            let selectedBuds = selectBudsWithSpace()
            terminalBudCount = 0
            lateralBudCount = 0
            var result: [Bud] = []
            for (bud, _) in selectedBuds {
                result.append(bud)
                if bud is TerminalBud {
                    terminalBudCount += 1
                } else if bud is LateralBud {
                    lateralBudCount += 1
                }
            }
            guard attractionPoints.count > 0 else { throw Error.noAttractionPoints }
            guard selectedBuds.count > 0 else { throw Error.noSelectedBuds }

            let exposures = updateLightExposure()

            let vigors = updateVigor(exposures: exposures)
            var maxVigor: Float = 0
            for (bud, _) in selectedBuds {
                maxVigor = max(maxVigor, vigors[bud]!)
            }
            guard maxVigor > 0 else { throw Error.noVigor }

            growShoots(selectedBuds: selectedBuds, vigors: vigors, maxVigor: maxVigor)
            generation += 1
            if enableAllBuds {
                self.attractionPoints = []
            }
            return result
        }
    }
}
