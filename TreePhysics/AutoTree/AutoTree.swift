import Foundation
import simd
import SceneKit

struct AutoTree {
    let config: AutoTreeConfig
    let simulator: GrowthSimulator

    init(_ config: AutoTreeConfig = AutoTreeConfig()) {
        self.config = config
        self.simulator = GrowthSimulator(config)
    }

    func root() -> Node {
        return Node(config: config, position: .zero, orientation: .identity)
    }

    func lateralBud(position: float3, orientation: simd_quatf) -> LateralBud {
        return LateralBud(config: config, position: position, orientation: orientation)
    }

    func terminalBud(position: float3, orientation: simd_quatf) -> TerminalBud {
        return TerminalBud(config: config, position: position, orientation: orientation)
    }

    func seedling(position: float3 = .zero, orientation: simd_quatf = .identity) -> Node {
        let root = self.root()
        let bud = self.terminalBud(position: .zero, orientation: .identity)
        root.addChild(bud)
        return root
    }

    func growthSimulator() -> GrowthSimulator {
        return GrowthSimulator(config)
    }

    func draw(_ node: Node, pen: CylinderPen<UInt16>, level: Int = 0) {
        switch node {
        case let bud as AutoTree.Bud:
            _ = pen.copy(scale: 0.01, orientation: bud.orientation)
        case let internode as AutoTree.Internode:
            _ = pen.cont(distance: internode.length, orientation: internode.orientation, thickness: sqr(internode.radius) * .pi)
        default:
            pen.start(at: node.position, thickness: 1)
        }

        // Reorganize branching structure following thickest path topology,
        // cf, [Longay 2014], appendix C.3
        let (thickest, rest) = node.children.thickest
        for child in rest {
            let branch = pen.branch()
            _ = draw(child, pen: branch, level: level + 1)
        }
        if let thickest = thickest { _ = draw(thickest, pen: pen) }
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
            case let (.some(last), internode as AutoTree.Internode) where internode.radius > last.radius:
                rest.insert(last)
                thickest = internode
            default:
                rest.insert(node)
            }
        }
        return (thickest, rest)
    }
}
