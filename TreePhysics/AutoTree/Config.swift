import Foundation

extension AutoTree {
    public struct Config {
        public var internodeLength: Float = 0.05

        public var branchStraightnessBias: Float = 0
        public var branchGravitropismBias: Float = 0
        public var branchEnvironmentalBias: Float { return 1 - branchStraightnessBias - branchGravitropismBias }

        public var branchingAngle: Float = .pi/8
        public var phyllotacticAngle: Float = .pi/4
        public var gravitropismAngle: Float = 0

        public var occupationRadius: Float = 0.05
        public var perceptionAngle: Float = .pi/2
        public var perceptionRadius: Float = 0.05 * 12

        public var meshDetail: Float = 1.1
        public var extremityRadius: Float = 0.001
        public var baseRadius: Float = 0.05

        public var fullExposure: Float = 100
        public var sensitivityOfBudsToLight: Float = 1.1
        public var apicalDominance: Float = 0.5
        public var maxShootLength: Float = 3.5

        public var shadowIntensity: Float = 0.1
        public var shadowDecayFactor: Float = 0.5
        public var shadowDepth: Int = 4
        public var initialShadowGridSize: Int = 256

        public var verticalGravimorphismBias: Float = 1
        public var horizontalGravimorphismBias: Float = 1
        public var upperSideGravimorphismBias: Float = 0

        public init() {}

        public func validate() {
            check.positive(internodeLength)

            check.normal(branchStraightnessBias)
            check.normal(branchGravitropismBias)
            check.normal(branchStraightnessBias + branchGravitropismBias)

            check.angle(branchingAngle)
            check.angle(phyllotacticAngle)
            check.angle(perceptionAngle)
            check.angle(gravitropismAngle)

            check.positive(occupationRadius)
            check.positive(perceptionRadius)
            check.assert(meshDetail > 1)
            check.positive(extremityRadius)
            check.positive(baseRadius)

            check.positive(fullExposure)
            check.nonneg(sensitivityOfBudsToLight)
            check.normal(apicalDominance)
            check.assert(maxShootLength >= 1)

            check.positive(shadowIntensity)
            check.normal(shadowDecayFactor)
            check.positive(shadowDepth)
            check.positive(initialShadowGridSize)

            check.positive(verticalGravimorphismBias)
            check.positive(horizontalGravimorphismBias)
            check.assert(abs(upperSideGravimorphismBias) < verticalGravimorphismBias)
        }
    }
}
