import Foundation
import simd
import SceneKit

struct AutoTree {
    let config: AutoTreeConfig

    init(_ config: AutoTreeConfig = AutoTreeConfig()) {
        self.config = config
    }

    func root() -> Parent {
        return Parent(config: config, position: .zero, orientation: .identity)
    }

    func lateralBud(position: float3, orientation: simd_quatf) -> LateralBud {
        return LateralBud(config: config, position: position, orientation: orientation)
    }

    func terminalBud(position: float3, orientation: simd_quatf) -> TerminalBud {
        return TerminalBud(config: config, position: position, orientation: orientation)
    }

    func internode(position: float3, orientation: simd_quatf) -> Internode {
        return Internode(config: config, position: position, orientation: orientation)
    }

    func seedling(position: float3 = .zero, orientation: simd_quatf = .identity) -> (Parent, TerminalBud) {
        let root = self.root()
        let bud = self.terminalBud(position: .zero, orientation: .identity)
        root.addBud(bud)
        return (root, bud)
    }

    func growthSimulator(shadowGrid: ShadowGrid? = nil) -> GrowthSimulator {
        let shadowGrid = shadowGrid ?? HashingShadowGrid(ShadowGridConfig(cellSize: config.internodeLength))
        return GrowthSimulator(config, shadowGrid: shadowGrid)
    }

    func draw(_ parent: Parent, pen: CylinderPen<UInt16>) {
        let diameterExponent = log(Float(parent.terminalBranchCount)) / (log(2*config.baseRadius) - log(2*config.extremityRadius))
        draw(parent, pen: pen, diameterExponent: diameterExponent)
    }

    func draw(_ node: Node, pen: CylinderPen<UInt16>, diameterExponent: Float) {
        switch node {
        case let bud as Bud:
            _ = pen.copy(scale: 0.01, orientation: bud.orientation)
        case let internode as Internode:
            let diameter = internode.diameter(exponent: diameterExponent)
            let thickness = sqr(diameter / 2) * .pi
            _ = pen.cont(distance: config.internodeLength, orientation: internode.orientation, thickness: thickness)
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
                radialSegmentCount = Int(3 * pow(internode.diameter(exponent: diameterExponent), config.n) / config.extremityRadius)
            } else {
                radialSegmentCount = nil
            }

            let branch = pen.branch(radialSegmentCount: max(3, radialSegmentCount ?? 3))
            _ = draw(child, pen: branch, diameterExponent: diameterExponent)
        }
        if let thickest = thickest { _ = draw(thickest, pen: pen, diameterExponent: diameterExponent) }
    }
}

fileprivate extension Set where Element == AutoTree.Node {
    var thickest: (AutoTree.Node?, Set<AutoTree.Node>) {
        var rest: Set<AutoTree.Node> = []
        var thickest: AutoTree.Internode? = nil
        for node in self {
            switch (thickest, node) {
            case let (nil, internode as AutoTree.Internode):
                thickest = internode
            case let (.some(last), internode as AutoTree.Internode) where internode.terminalBranchCount > last.terminalBranchCount:
                rest.insert(last)
                thickest = internode
            default:
                rest.insert(node)
            }
        }
        return (thickest, rest)
    }
}
