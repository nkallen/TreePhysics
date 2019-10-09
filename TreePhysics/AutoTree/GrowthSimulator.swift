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
}

extension AutoTree {
    final class GrowthSimulator {
        let config: AutoTreeConfig
        var attractionPoints: Set<float3> = []
        let hash: LocalitySensitiveHash<Bud>
        let shadowGrid: ShadowGrid
        var buds: Set<Bud> = []

        init(_ config: AutoTreeConfig, shadowGrid: ShadowGrid) {
            self.config = config
            self.hash = LocalitySensitiveHash<Bud>(cellSize: config.occupationRadius + config.perceptionRadius)
            self.shadowGrid = shadowGrid
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

            if let mainChild = node.mainChild {
                add(mainChild)
            }
            if let lateralChild = node.lateralChild {
                add(lateralChild)
            }
        }

        internal func updateLightExposure() -> [Internode:Float] {
            var lightExposures: [Internode:Float] = [:]
            let start = Date()
            print("====", buds.count)
            for bud in buds {
                let lightExposure = self.lightExposure(of: bud)
                var node: Node = bud
                while let internode = node.parent as? Internode {
                    lightExposures[internode] = lightExposures[internode, default: 0] + lightExposure
                    print("setting to \(lightExposures[internode])) given \(lightExposure)")
                    node = internode
                }
            }
            print("finishsed in ", Date().timeIntervalSince(start))
            return lightExposures
        }

//        internal func updateVigor(exposures: [Internode:Float]) -> [Internode:Float] {
//            let main: Internode = ()
//            let lateral: Internode = ()
//
//            let baseVigor = vigor(for: root, given: exposures)
//            let mainQ = vigor(for: main, given: exposures)
//            let lateralQ = vigor(for: lateral, given: exposures)
//
//            let denominator = (config.lambda * mainQ + (1 - config.lambda) * lateralQ) / baseVigor
//            let mainVigor = config.lambda * mainQ / denominator
//            let lateralVigor = (1 - config.lambda) * lateralQ / denominator
//        }
//
//        private func vigor(for internode: Internode, given exposures: [Internode:Float]) -> Float {
//            let l = exposures[root]
//            return pow(l, config.k)
//        }

        func update() {
            let exposures = updateLightExposure()
//            _ = updateVigor(exposures: exposures)

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

        private func lightExposure(of bud: Bud) -> Float {
            return max(config.fullExposure - shadowGrid[bud.position] + config.shadowIntensity, 0)
        }
    }
}
