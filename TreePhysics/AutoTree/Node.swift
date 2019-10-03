import Foundation
import simd

fileprivate let branchingAngle: Float = .pi/4
fileprivate let phyllotacticAngle: Float = .pi/4
fileprivate let length: Float = 1
fileprivate let radius: Float = 1

fileprivate let occupationRadius: Float = 1
fileprivate let perceptionAngle: Float = 1
fileprivate let perceptionRadius: Float = 1

extension AutoTree.Bud: HasPosition {}
extension AutoTree.Bud: Hashable {
    func hash(into hasher: inout Hasher) {
        fatalError()
    }
}

extension AutoTree {
    enum Kind {
        case bud(position: float3, orientation: simd_quatf)
        case internode(position: float3, orientation: simd_quatf)
    }

    struct Bud {
        let position: float3
        let orientation: simd_quatf
        let producesLateralBud: Bool

        func grow(towards points: [float3]) -> Node {
            var newDirection = float3.zero
            for point in points {
                let directionToAttractionPoint = point - self.position
                newDirection += normalize(directionToAttractionPoint)
            }
            let rotation = simd_quatf(from: orientation.heading, to: normalize(newDirection))
            return Node(
                position: self.position,
                orientation: (rotation * self.orientation).normalized,
                hasLateralBud: producesLateralBud)
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

    struct Node {
        let position: float3
        let orientation: simd_quatf

        var lateralBud: Bud?
        var terminalBud: Bud?

        init(position: float3, orientation: simd_quatf, hasLateralBud: Bool) {
            self.position = position
            self.orientation = orientation

            let branchingRotation = simd_quatf(angle: branchingAngle, axis: orientation.up)
            let phyllotacticRotation = simd_quatf(angle: phyllotacticAngle, axis: orientation.heading)

            if hasLateralBud {
                self.lateralBud = Bud(
                    position: self.position + phyllotacticRotation.act(orientation.left) * radius,
                    orientation: (phyllotacticRotation * branchingRotation * orientation).normalized,
                    producesLateralBud: false)
            } else {
                self.lateralBud = nil
            }

            self.terminalBud = Bud(
                position: position + orientation.heading * length,
                orientation: orientation,
                producesLateralBud: true)
        }
    }

    class GrowthSimulator {
        var attractionPoints: Set<float3> = []
        let hash = LocalitySensitiveHash<Bud>(cellSize: perceptionRadius)

        func update() {
            var selectedBuds: [Bud:NSMutableSet] = [:]

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
                    let attractionPointsForBud: NSMutableSet
                    if let ps = selectedBuds[closestBud] {
                        attractionPointsForBud = ps
                    } else {
                        attractionPointsForBud = NSMutableSet()
                        selectedBuds[closestBud] = attractionPointsForBud
                    }
                    let node = closestBud.grow(towards: attractionPointsForBud.allObjects as! [float3])
                    hash.remove(closestBud)
                    hash.add(node.terminalBud!)
                    if let lateralBud = node.lateralBud { hash.add(lateralBud) }
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
