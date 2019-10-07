import Foundation
import SceneKit
import ModelIO

public final class CylinderPen<I>: Pen where I: FixedWidthInteger {
    public typealias T = [I]
    private struct State {
        let position: float3
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

    public func start(at position: float3, orientation: simd_quatf, thickness: Float) {
        let bottom = makeCircle(radius: sqrt(thickness / .pi)).map {
            position + orientation.act($0)
        }
        let vertexId = branchGeometry.addVertices(bottom)

        state = State(position: position, orientation: orientation, thickness: thickness, vertexId: vertexId)
    }

    public func cont(distance: Float, orientation: simd_quatf, thickness: Float) -> T {
        guard let state = state else { fatalError() }

        let top: [float3] = makeCircle(radius: sqrt(thickness / .pi)).map {
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

        let vertices = [float3(-0.5, 0, 0), float3(0, 1, 0),
                        float3(0, 1, 0), float3(0.5, 0, 0)]
        let indices: [I] = [0,1,2, 0,2,3,
                            2,1,0, 3,2,0]

        let rotatedVertices: [float3]
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

    private func makeCircle(radius: Float) -> [float3] {
        var vertices: [float3] = []
        let arcLength: Float = 2.0 * .pi / Float(radialSegmentCount)
        for j in 0..<radialSegmentCount {
            let theta = arcLength*Float(j)
            let rCosTheta = radius*cos(theta)
            let rSinTheta = radius*sin(theta)

            vertices.append(float3(rCosTheta, 0, rSinTheta))
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

final class GeometryBuilder<I> where I: FixedWidthInteger {
    private let primitiveType: SCNGeometryPrimitiveType
    var vertices: [float3] = []
    var indices: [I] = []

    init(primitiveType: SCNGeometryPrimitiveType) {
        self.primitiveType = primitiveType
    }

    func addVertices(_ vertices: [float3]) -> I {
        let minVertexId = self.vertices.count
        self.vertices.append(contentsOf: vertices)
        return I(minVertexId)
    }

    func addIndices(_ indices: [I]) {
        if primitiveType == .triangleStrip, let last = self.indices.last {
            let degenerate = [last, indices.first!]
            self.indices.append(contentsOf: degenerate)
        }

        self.indices.append(contentsOf: indices)
    }

    func addSegment(_ vertices: [float3], _ indices: [I]) -> [I] {
        let offsetIndices = indices.map { I(vertices.count) + $0 }

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

    private(set) lazy var geometry: SCNGeometry = {
        return SCNGeometry(sources: [source], elements: [element])
    }()
}
