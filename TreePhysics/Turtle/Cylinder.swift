import Foundation
import simd
import SceneKit

class CylinderPen {
    private(set) var vertices: [float3] = []
    private(set) var indices: [UInt16] = []

    private var start: float2? = nil

    let radialSegmentCount: Int
    let heightSegmentCount: Int

    init(radialSegmentCount: Int = 48, heightSegmentCount: Int = 1) {
        guard radialSegmentCount >= 3 else { fatalError() }

        self.radialSegmentCount = radialSegmentCount
        self.heightSegmentCount = heightSegmentCount
    }

    // does start need a tangent?
    func start(at: float2, thickness: Float) {
        start = at
    }

    func cont(distance: Float, tangent: float2, thickness: Float) {
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
        let offset = UInt16(self.vertices.count)
        let offsetIndices = indices.map { offset + $0 }

        if let last = self.indices.last {
            let degenerate = [last, offsetIndices.first!]
            self.indices.append(contentsOf: degenerate)
        }

        self.vertices.append(contentsOf: rotatedVertices)
        self.indices.append(contentsOf: offsetIndices)

        self.start = start + distance * tangent
    }

    private func makeSegment(radius: Float, height: Float) -> ([float3], [UInt16]) {
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

    var element: SCNGeometryElement {
        print("indices", indices)
        return SCNGeometryElement(indices: indices, primitiveType: .triangleStrip)
    }

    var source: SCNGeometrySource {
        print("vertices", vertices)
        return SCNGeometrySource(vertices: vertices.map { SCNVector3($0) })
    }
}
