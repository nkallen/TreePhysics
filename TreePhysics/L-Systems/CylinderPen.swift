import Foundation
import SceneKit
import ModelIO

// NOTE: We need to keep all the vertices to gether in one array; however, we can keep the
// indices separate so that there are distinct SCNGeometryElement objects. It's unclear how this
// affects rendering performance however.

final class CylinderPen: Pen {
    typealias T = Indices

    private(set) var vertices: [float3] = []
    private(set) var indices: Indices = []
    weak var parent: CylinderPen?

    private var start: float3? = nil

    let radialSegmentCount: Int
    let heightSegmentCount: Int

    init(radialSegmentCount: Int = 48, heightSegmentCount: Int = 1, parent: CylinderPen? = nil) {
        guard radialSegmentCount >= 3 else { fatalError() }

        self.radialSegmentCount = radialSegmentCount
        self.heightSegmentCount = heightSegmentCount
        self.parent = parent
    }

    func start(at: float3, thickness: Float) {
        start = at
    }

    func cont(distance: Float, tangent: float3, thickness: Float) -> Indices {
        precondition(length(tangent) > 0)
        guard let start = start else { fatalError() }

        let radius = sqrt(thickness / .pi)

        let (vertices, indices) = makeSegment(radius: radius, height: distance)

        let rotation = simd_quatf(from: float3(0,1,0), to: tangent)
        let rotatedVertices: [float3]
        rotatedVertices = vertices.map { vertex in
            start + rotation.act(vertex)
        }

        self.start = start + distance * tangent

        return addSegment(rotatedVertices, indices)
    }

    func copy(scale: Float, orientation: simd_quatf) -> Indices {
        guard let start = start else { fatalError() }

        let sphere = MDLMesh.newEllipsoid(withRadii: float3(repeating: scale), radialSegments: 10, verticalSegments: 10, geometryType: .triangleStrips, inwardNormals: false, hemisphere: false, allocator: nil)
        let submesh = sphere.submeshes?.firstObject! as! MDLSubmesh
        let verticesPointer = sphere.vertexBuffers.first!.map().bytes.bindMemory(to: float3.self, capacity: sphere.vertexCount)
        let indicesPointer = submesh.indexBuffer.map().bytes.bindMemory(to: UInt16.self, capacity: submesh.indexCount)

        let vertices = Array(UnsafeBufferPointer(start: verticesPointer, count: sphere.vertexCount))
        let indices = Array(UnsafeBufferPointer(start: indicesPointer, count: submesh.indexCount))

        let rotatedVertices: [float3]
        rotatedVertices = vertices.map { vertex in
            start + orientation.act(vertex)
        }

        return addSegment(rotatedVertices, indices)
    }

    func addSegment(_ vertices: [float3], _ indices: Indices) -> Indices {
        if let parent = parent {
            return parent.addSegment(vertices, indices)
        } else {
            let offset = UInt16(self.vertices.count)
            let offsetIndices = indices.map { offset + $0 }

            if let last = self.indices.last {
                let degenerate = [last, offsetIndices.first!]
                self.indices.append(contentsOf: degenerate)
            }

            self.vertices.append(contentsOf: vertices)
            self.indices.append(contentsOf: offsetIndices)

            return Array(Set(offsetIndices))
        }
    }

    var branch: CylinderPen {
        guard let start = start else { fatalError() }
        let pen = CylinderPen(radialSegmentCount: radialSegmentCount, heightSegmentCount: heightSegmentCount, parent: self)
        pen.start(at: start, thickness: 1)
        return pen
    }

    private func makeSegment(radius: Float, height: Float) -> ([float3], Indices) {
        var vertices: [float3] = []
        let radialSegmentCountUInt16 = UInt16(radialSegmentCount)
        let arcLength: Float = 2.0 * .pi / Float(radialSegmentCount)
        for j in 0..<radialSegmentCountUInt16 {
            let theta = arcLength*Float(j)
            let rCosTheta = radius*cos(theta)
            let rSinTheta = radius*sin(theta)

            vertices.append(float3(rCosTheta, 0, rSinTheta)) // XXX can skip in subsequent
            vertices.append(float3(rCosTheta, height, rSinTheta))
        }
        let indices = Array(0..<(radialSegmentCountUInt16*2)) + [0,1]
        return (vertices, indices)
    }

    private(set) lazy var element: SCNGeometryElement = {
        return SCNGeometryElement(indices: indices, primitiveType: .triangleStrip)
    }()

    private(set) lazy var source: SCNGeometrySource = {
        return SCNGeometrySource(vertices: vertices.map { SCNVector3($0) })
    }()

    private(set) lazy var geometry: SCNGeometry = {
        return SCNGeometry(sources: [source], elements: [element])
    }()
}
