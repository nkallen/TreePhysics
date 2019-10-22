import Foundation
import SceneKit
import ModelIO

public final class CylinderPen<I>: Pen where I: FixedWidthInteger {
    public typealias T = [I]
    private struct State {
        let position: SIMD3<Float>
        let orientation: simd_quatf
        let thickness: Float
        let vertexId: I
    }
    private var state: State? = nil

    let radialSegmentCount: Int

    let branchGeometry: GeometryBuilder<I>
    let leafGeometry: GeometryBuilder<I>

    weak var parent: CylinderPen?

    public init(radialSegmentCount: Int = 3, parent: CylinderPen? = nil) {
        self.parent = parent
        self.radialSegmentCount = radialSegmentCount

        if let parent = parent {
            self.branchGeometry = parent.branchGeometry
            self.leafGeometry = parent.leafGeometry
        } else {
            self.branchGeometry = GeometryBuilder(primitiveType: .triangleStrip)
            self.leafGeometry = GeometryBuilder(primitiveType: .triangles)
        }
    }

    public func start(at position: SIMD3<Float>, orientation: simd_quatf, thickness: Float) {
        let bottom = makeCircle(radius: sqrt(thickness / .pi)).map {
            position + orientation.act($0)
        }
        let vertexId = branchGeometry.addVertices(bottom)

        state = State(position: position, orientation: orientation, thickness: thickness, vertexId: vertexId)
    }

    public func cont(distance: Float, heading: SIMD3<Float>, thickness: Float) -> T {
        guard let state = state else { fatalError() }
        let turn = simd_quatf(from: state.orientation.heading, to: heading)
        return cont(distance: distance, orientation: (turn * state.orientation).normalized, thickness: thickness)
    }

    public func cont(distance: Float, orientation: simd_quatf, thickness: Float) -> T {
        guard let state = state else { fatalError() }

        let circle = makeCircle(radius: sqrt(thickness / .pi))
        let top: [SIMD3<Float>] = circle.map {
            state.position + orientation.act(distance * .y + $0)
        }

        let topVertexId = branchGeometry.addVertices(top)

        var indices: [I] = (0..<I(radialSegmentCount)).flatMap { [state.vertexId + $0, topVertexId + $0] }
        indices += [state.vertexId, topVertexId]
        branchGeometry.addIndices(indices)

        self.state = State(position: state.position + distance * orientation.heading, orientation: orientation, thickness: thickness, vertexId: topVertexId)
        return Array(Set(indices))
    }

    public func copy(scale: Float, orientation: simd_quatf) -> T {
        guard let state = state else { fatalError() }

        let vertices = [SIMD3<Float>(-0.5, 0, 0), SIMD3<Float>(0, 2, 0),
                        SIMD3<Float>(0, 2, 0), SIMD3<Float>(0.5, 0, 0)]
        let indices: [I] = [0,1,2, 0,2,3,
                            2,1,0, 3,2,0]

        let rotatedVertices: [SIMD3<Float>]
        rotatedVertices = vertices.map { vertex in
            state.position + orientation.act(scale * vertex)
        }

        return leafGeometry.addSegment(rotatedVertices, indices)
    }

    public func branch() -> CylinderPen<I> {
        return branch(radialSegmentCount: nil)
    }

    public func branch(radialSegmentCount: Int? = nil) -> CylinderPen<I> {
        guard let state = state else { fatalError() }
        let pen = CylinderPen<I>(radialSegmentCount: radialSegmentCount ?? self.radialSegmentCount, parent: self)
        pen.start(at: state.position, orientation: state.orientation, thickness: state.thickness)
        return pen
    }

    private func makeCircle(radius: Float) -> [SIMD3<Float>] {
        var vertices: [SIMD3<Float>] = []
        let arcLength: Float = 2.0 * .pi / Float(radialSegmentCount)
        for j in 0..<radialSegmentCount {
            let theta = arcLength*Float(j)
            let rCosTheta = radius*cos(theta)
            let rSinTheta = radius*sin(theta)

            vertices.append(SIMD3<Float>(rCosTheta, 0, rSinTheta))
        }
        return vertices
    }

    func node() -> SCNNode {
        let parent = SCNNode()
        let leaves = SCNNode(geometry: leafGeometry.geometry)
        leaves.geometry?.firstMaterial!.diffuse.contents = NSColor(red: 1, green: 0.718, blue: 0.773, alpha: 0.9)
        let branches = SCNNode(geometry: branchGeometry.geometry)
        parent.addChildNode(leaves)
        parent.addChildNode(branches)
        return parent
    }
}

public final class GeometryBuilder<I> where I: FixedWidthInteger {
    private let primitiveType: SCNGeometryPrimitiveType
    private let material: SCNMaterial?
    var vertices: [SIMD3<Float>] = []
    var indices: [I] = []

    public init(primitiveType: SCNGeometryPrimitiveType, material: SCNMaterial? = nil) {
        self.primitiveType = primitiveType
        self.material = material
    }

    public func addVertices(_ vertices: [SIMD3<Float>]) -> I {
        let minVertexId = self.vertices.count
        self.vertices.append(contentsOf: vertices)
        return I(minVertexId)
    }

    public func addIndices(_ indices: [I]) {
        if primitiveType == .triangleStrip, let last = self.indices.last {
            let degenerate = [last, indices.first!]
            self.indices.append(contentsOf: degenerate)
        }

        self.indices.append(contentsOf: indices)
    }

    public func addSegment(_ vertices: [SIMD3<Float>], _ indices: [I]) -> [I] {
        let offset = I(self.vertices.count)
        let offsetIndices = indices.map { offset + $0 }

        _ = addVertices(vertices)
        addIndices(offsetIndices)

        return Array(Set(offsetIndices))
    }

    private(set) lazy var element: SCNGeometryElement = {
        return SCNGeometryElement(indices: indices, primitiveType: primitiveType)
    }()

    private(set) lazy var source: SCNGeometrySource = {
        return SCNGeometrySource(vertices: vertices.map { SCNVector3($0) })
    }()

    public private(set) lazy var geometry: SCNGeometry = {
        let result = SCNGeometry(sources: [source], elements: [element])
        if let material = material {
            result.firstMaterial = material
        }
        return result
    }()
}
