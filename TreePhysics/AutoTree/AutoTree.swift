import Foundation
import simd
import SceneKit

public struct AutoTree {
    let config: Config
    
    public init(_ config: Config = Config()) {
        config.validate()
        self.config = config
    }
    
    func root(position: SIMD3<Float> = .zero) -> Parent {
        return Parent(config: config, position: position, orientation: .identity)
    }
    
    public func lateralBud(position: SIMD3<Float>, orientation: simd_quatf) -> LateralBud {
        return LateralBud(config: config, position: position, orientation: orientation)
    }
    
    public func terminalBud(position: SIMD3<Float>, orientation: simd_quatf) -> TerminalBud {
        return TerminalBud(config: config, position: position, orientation: orientation)
    }
    
    public func internode(position: SIMD3<Float>, orientation: simd_quatf) -> Internode {
        return Internode(config: config, position: position, orientation: orientation)
    }
    
    public func seedling(position: SIMD3<Float> = .zero, orientation: simd_quatf = .identity) -> (Parent, TerminalBud) {
        let root = self.root(position: position)
        let bud = self.terminalBud(position: position, orientation: orientation)
        root.addBud(bud)
        return (root, bud)
    }
    
    public func growthSimulator(shadowGrid: ShadowGrid? = nil) -> GrowthSimulator {
        let shadowGrid = shadowGrid ?? ArrayBackedShadowGrid(config)
        return GrowthSimulator(config, shadowGrid: shadowGrid)
    }
    
    public func draw<I: FixedWidthInteger>(_ parent: Parent, pen: CylinderPen<I>, showBuds: Bool = false) {
        let diameterExponent = log(Float(parent.terminalBudCount)) / (log(2*config.baseRadius) - log(2*config.extremityRadius))
        draw(parent, pen: pen, diameterExponent: diameterExponent, showBuds: showBuds)
    }
    
    func draw<I: FixedWidthInteger>(_ node: Node, pen: CylinderPen<I>, diameterExponent: Float, showBuds: Bool = false) {
        switch node {
        case let bud as Bud:
            if showBuds {
                _ = pen.copy(scale: config.internodeLength, orientation: bud.orientation)
            }
            if case let lateralBud as LateralBud = bud {
                let internode = lateralBud.parent as! Internode
                let diameter = internode.diameter(exponent: diameterExponent)
                let radius = diameter / 2
                if radius <= config.leafRadiusThreshold {
                    let branch = pen.branch(radialSegmentCount: 3)
                    drawPetiole(from: bud.orientation, pen: branch)
                }
            }
        case let internode as Internode:
            // FIXME diameter math seems wrong
            let diameter = internode.diameter(exponent: diameterExponent)
            let thickness = sqr(diameter / 2) * .pi
            _ = pen.cont(distance: config.internodeLength, heading: internode.orientation.heading, thickness: thickness)

        default:
            pen.start(at: node.position, orientation: node.orientation, thickness: sqr(config.baseRadius) * .pi)
        }
        
        guard let parent = node as? Parent else { return }
        // Reorganize branching structure following thickest path topology,
        // cf, [Longay 2014], appendix C.3
        let (thickest, rest) = parent.thickestChild
        for child in rest {
            let radialSegmentCount: Int?
            if case let internode as Internode = child {
                radialSegmentCount = Int(3 * pow(internode.diameter(exponent: diameterExponent), config.meshDetail) / config.extremityRadius)
            } else {
                radialSegmentCount = nil
            }

            // FIXME extract constant for 3
            let branch = pen.branch(radialSegmentCount: max(3, radialSegmentCount ?? 3))
            _ = draw(child, pen: branch, diameterExponent: diameterExponent, showBuds: showBuds)
        }
        if let thickest = thickest { _ = draw(thickest, pen: pen, diameterExponent: diameterExponent, showBuds: showBuds) }
    }

    private func drawPetiole<I: FixedWidthInteger>(from orientation: simd_quatf, pen: CylinderPen<I>) {
        let leafTropismDirection = simd_quatf(angle: config.leafTropismAngle, axis: .x).act(.y)

        var orientation = orientation
        for _ in 0..<config.petioleSegmentCount {
            let headingRateOfRotation = -config.eh * dot(orientation.left, leafTropismDirection)
            let leftRateOfRotation = config.eh * dot(orientation.heading, leafTropismDirection)
            let upRateOfRotation = config.eh * dot(orientation.left, cross(leafTropismDirection, cross(orientation.heading, leafTropismDirection)))

            let sora = orientation.act(simd_float3(leftRateOfRotation, headingRateOfRotation, upRateOfRotation))
            let rotation = simd_length(sora) < 10e-10 ? simd_quatf.identity : simd_quatf(angle: simd_length(sora), axis: normalize(sora))

            orientation = rotation * orientation

            _ = pen.cont(distance: config.petioleLength / Float(config.petioleSegmentCount), orientation: orientation, thickness: sqr(config.petioleRadius) * .pi)
        }
        _ = pen.copy(scale: config.leafScale, orientation: orientation)
    }
}
