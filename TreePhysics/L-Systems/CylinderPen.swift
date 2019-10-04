import Foundation
import SceneKit
import ModelIO

public final class CylinderPen<I>: Pen where I: FixedWidthInteger {
    public typealias T = [I]

    let branchGeometry: GeometryBuilder<I>
    let leafGeometry: GeometryBuilder<I>

    weak var parent: CylinderPen?

    private var start: float3? = nil

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

    public func start(at: float3, thickness: Float) {
        start = at
    }

    public func cont(distance: Float, orientation: simd_quatf, thickness: Float) -> T {
        guard let start = start else { fatalError() }

        let radius = sqrt(thickness / .pi)

        let (vertices, indices) = makeSegment(radius: radius, height: distance)

        let rotatedVertices: [float3]
        rotatedVertices = vertices.map { vertex in
            start + orientation.act(vertex)
        }

        self.start = start + distance * orientation.heading

        return branchGeometry.addSegment(rotatedVertices, indices)
    }

    public func copy(scale: Float, orientation: simd_quatf) -> T {
        guard let start = start else { fatalError() }

        let vertices = [float3(-0.5, 0, 0), float3(-0.5, 1, 0),
                        float3(0.5, 1, 0), float3(0.5, 0, 0)]
        let indices: T = [0,1,2, 0,2,3,
                            2,1,0, 3,2,0]

        let rotatedVertices: [float3]
        rotatedVertices = vertices.map { vertex in
            start + orientation.act(scale * vertex)
        }

        return leafGeometry.addSegment(rotatedVertices, indices)
    }

    public func branch() -> CylinderPen<I> {
        guard let start = start else { fatalError() }
        let pen = CylinderPen<I>(radialSegmentCount: radialSegmentCount, heightSegmentCount: heightSegmentCount, parent: self)
        pen.start(at: start, thickness: 1)
        return pen
    }

    private func makeSegment(radius: Float, height: Float) -> ([float3], T) {
        var vertices: [float3] = []
        let arcLength: Float = 2.0 * .pi / Float(radialSegmentCount)
        for j in 0..<radialSegmentCount {
            let theta = arcLength*Float(j)
            let rCosTheta = radius*cos(theta)
            let rSinTheta = radius*sin(theta)

            vertices.append(float3(rCosTheta, 0, rSinTheta)) // FIXME can skip in subsequent
            vertices.append(float3(rCosTheta, height, rSinTheta))
        }
        let indices = Array(0..<(I(radialSegmentCount*2))) + [0,1]
        return (vertices, indices)
    }

    var node: SCNNode {
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

    func addSegment(_ vertices: [float3], _ indices: [I]) -> [I] {
        let offset = I(self.vertices.count)
        let offsetIndices = indices.map { offset + $0 }

        if primitiveType == .triangleStrip, let last = self.indices.last {
            let degenerate = [last, offsetIndices.first!]
            self.indices.append(contentsOf: degenerate)
        }

        self.vertices.append(contentsOf: vertices)
        self.indices.append(contentsOf: offsetIndices)

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
