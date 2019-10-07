import Foundation
import SceneKit
import ModelIO

public final class CylinderPen<I>: Pen where I: FixedWidthInteger {
    public typealias T = [I]
    private struct State {
        let position: float3
        let orientation: simd_quatf
        let thickness: Float
    }

    let branchGeometry: GeometryBuilder<I>
    let leafGeometry: GeometryBuilder<I>

    weak var parent: CylinderPen?

    private var state: State? = nil

    let radialSegmentCount: Int
    let heightSegmentCount: Int

    public init(radialSegmentCount: Int = 48, heightSegmentCount: Int = 1, parent: CylinderPen? = nil) {
        guard radialSegmentCount >= 3 else { fatalError() }

        self.radialSegmentCount = radialSegmentCount
        self.heightSegmentCount = heightSegmentCount
        self.parent = parent

        if let parent = parent {
            self.branchGeometry = parent.branchGeometry
            self.leafGeometry = parent.leafGeometry
        } else {
            self.branchGeometry = GeometryBuilder(primitiveType: .triangleStrip)
            self.leafGeometry = GeometryBuilder(primitiveType: .triangles)
        }
    }

    public func start(at: float3, orientation: simd_quatf, thickness: Float) {
        state = State(position: at, orientation: orientation, thickness: thickness)
    }

    public func cont(distance: Float, orientation: simd_quatf, thickness: Float) -> T {
        guard let state = state else { fatalError() }

        let bottom = makeCircle(radius: sqrt(state.thickness / .pi)).map {
            state.position + state.orientation.act($0)
        }

        let top = makeCircle(radius: sqrt(thickness / .pi)).map {
            state.position + orientation.act(distance * .y + $0)
        }

        self.state = State(position: state.position + distance * orientation.heading, orientation: orientation, thickness: thickness)

        return branchGeometry.addSegment(bottom + top, [0,3,1,4,2,5,0,3])
    }

    public func copy(scale: Float, orientation: simd_quatf) -> T {
        guard let state = state else { fatalError() }

        let vertices = [float3(-0.5, 0, 0), float3(0, 1, 0),
                        float3(0, 1, 0), float3(0.5, 0, 0)]
        let indices: T = [0,1,2, 0,2,3,
                            2,1,0, 3,2,0]

        let rotatedVertices: [float3]
        rotatedVertices = vertices.map { vertex in
            state.position + orientation.act(scale * vertex)
        }

        return leafGeometry.addSegment(rotatedVertices, indices)
    }

    public func branch() -> CylinderPen<I> {
        guard let state = state else { fatalError() }
        let pen = CylinderPen<I>(radialSegmentCount: radialSegmentCount, heightSegmentCount: heightSegmentCount, parent: self)
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
    var offset: I = 0

    init(primitiveType: SCNGeometryPrimitiveType) {
        self.primitiveType = primitiveType
    }

    func addSegment(_ vertices: [float3], _ indices: [I]) -> [I] {
        let offsetIndices = indices.map { offset + $0 }

        if primitiveType == .triangleStrip, let last = self.indices.last {
            let degenerate = [last, offsetIndices.first!]
            self.indices.append(contentsOf: degenerate)
        }

        self.vertices.append(contentsOf: vertices)
        self.indices.append(contentsOf: offsetIndices)
        self.offset += I(vertices.count)

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
