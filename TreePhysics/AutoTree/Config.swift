import Foundation

extension AutoTree {
    public struct Config {
        public var branchingAngle: Float = .pi/8
        public var phyllotacticAngle: Float = .pi/4
        public var internodeLength: Float = 0.05

        public var occupationRadius: Float = 0.05
        public var perceptionAngle: Float = .pi/4
        public var perceptionRadius: Float = 0.05 * 12

        public var meshDetail: Float = 1.1
        public var extremityRadius: Float = 0.001
        public var baseRadius: Float = 0.05

        public var fullExposure: Float = 10
        public var sensitivityOfBudsToLight: Float = 1.1
        public var biasVigorTowardsMainAxis: Float = 0.5
        public var maxShootLength: Int = 3

        public var shadowIntensity: Float = 0.1
        public var shadowDecayFactor: Float = 0.5
        public var shadowDepth: Int = 5
        public var initialShadowGridSize: Int = 200

        public init() {}

        public func validate() {
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
            check.nonneg(sensitivityOfBudsToLight)
            check.normal(biasVigorTowardsMainAxis)
            check.assert(maxShootLength >= 1)

            check.positive(shadowIntensity)
            check.normal(shadowDecayFactor)
            check.positive(shadowDepth)
            check.positive(initialShadowGridSize)
        }
    }
}
