import Foundation

extension AutoTree {
    struct Config {
        var branchingAngle: Float = .pi/4
        var phyllotacticAngle: Float = .pi/4
        var internodeLength: Float = 0.05

        var occupationRadius: Float = 0.05
        var perceptionAngle: Float = .pi/4
        var perceptionRadius: Float = 0.05 * 12

        var meshDetail: Float = 1.1
        var extremityRadius: Float = 0.001
        var baseRadius: Float = 0.05

        var fullExposure: Float = 10
        var shadowIntensity: Float = 0.1
        var sensitivityOfBudsToLight: Float = 1.1
        var biasVigorTowardsMainAxis: Float = 0.5
        var maxShootLength: Int = 3

        func validate() {
            check.angle(branchingAngle)
            check.angle(phyllotacticAngle)
            check.angle(perceptionAngle)

            check.positive(internodeLength)
            check.positive(occupationRadius)
            check.positive(perceptionRadius)
            check.assert(meshDetail > 1)
            check.positive(extremityRadius)
            check.positive(baseRadius)

            check.positive(fullExposure)
            check.positive(shadowIntensity)
            check.nonneg(sensitivityOfBudsToLight)
            check.normal(biasVigorTowardsMainAxis)
            check.assert(maxShootLength >= 1)
        }
    }
}
