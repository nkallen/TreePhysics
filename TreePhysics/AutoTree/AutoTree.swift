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
        case let internode as Internode:
            // FIXME diameter math is wrong
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
            
            let branch = pen.branch(radialSegmentCount: max(3, radialSegmentCount ?? 3))
            _ = draw(child, pen: branch, diameterExponent: diameterExponent, showBuds: showBuds)
        }
        if let thickest = thickest { _ = draw(thickest, pen: pen, diameterExponent: diameterExponent, showBuds: showBuds) }
    }
}
