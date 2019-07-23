import Foundation
import SceneKit

final class CylinderPen: Pen {
    typealias T = Indices

    private(set) var vertices: [float3] = []
    private(set) var indices: Indices = []
    weak var parent: CylinderPen?

    private var start: float2? = nil

    let radialSegmentCount: Int
    let heightSegmentCount: Int

    init(radialSegmentCount: Int = 48, heightSegmentCount: Int = 1, parent: CylinderPen? = nil) {
        guard radialSegmentCount >= 3 else { fatalError() }

        self.radialSegmentCount = radialSegmentCount
        self.heightSegmentCount = heightSegmentCount
        self.parent = parent
    }

    func start(at: float2, thickness: Float) {
        start = at
    }

    func cont(distance: Float, tangent: float2, thickness: Float) -> Indices {
        precondition(length(tangent) > 0)
        guard let start = start else { fatalError() }

        let radius = sqrt(thickness / .pi)

        let (vertices, indices) = makeSegment(radius: radius, height: distance)
        let rotation = matrix4x4_rotation(from: float3(0,1,0), to: float3(tangent, 0))
        let rotatedVertices: [float3]
        if tangent.x == 0 && tangent.y > 0 {
            rotatedVertices = vertices.map { vertex in
                float3(start, 0) + vertex
            }
        } else {
            rotatedVertices = vertices.map { vertex in
                float3(start, 0) + (rotation * float4(vertex, 0)).xyz
            }
        }

        self.start = start + distance * tangent

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
        print("indices", indices)
        return SCNGeometryElement(indices: indices, primitiveType: .triangleStrip)
    }()

    private(set) lazy var source: SCNGeometrySource = {
        print("vertices", vertices)
        return SCNGeometrySource(vertices: vertices.map { SCNVector3($0) })
    }()

    private(set) lazy var geometry: SCNGeometry = {
        return SCNGeometry(sources: [source], elements: [element])
    }()
}