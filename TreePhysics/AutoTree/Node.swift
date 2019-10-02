import Foundation
import simd

fileprivate let branchingAngle: Float = .pi/4
fileprivate let phyllotacticAngle: Float = .pi/4
fileprivate let length: Float = 1
fileprivate let radius: Float = 1

extension AutoTree {
    struct Bud {
        let position: float3
        let orientation: simd_quatf
        let producesLateralBud: Bool

        // occupationRadius: Float
        // perceptionAngle: Float
        // perceptionRadius: Float

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
