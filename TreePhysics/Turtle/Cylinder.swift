import Foundation
import simd
import SceneKit

class Cylinder {
    let vertices: [float3]
    let indices: [UInt16]

    init(radius: Float, height: Float, radialSegmentCount: Int = 48, heightSegmentCount: Int = 1) {
        guard radialSegmentCount >= 3 else { fatalError() }

        var vertices: [float3] = []
        let radialSegmentCountUInt16 = UInt16(radialSegmentCount)
        let arcLength: Float = 2.0 * .pi / Float(radialSegmentCount)
        let halfHeight: Float = height / 2.0
        //        for i in 0..<heightSegmentCount {
        for j in 0..<radialSegmentCountUInt16 {
            let theta = arcLength*Float(j)
            let rCosTheta = radius*cos(theta)
            let rSinTheta = radius*sin(theta)

            vertices.append(float3(rCosTheta, -halfHeight, rSinTheta))
            vertices.append(float3(rCosTheta, halfHeight, rSinTheta))
        }
//        }
        self.vertices = vertices
        self.indices = Array(0..<(radialSegmentCountUInt16*2)) + [0,1]
    }

    var element: SCNGeometryElement {
        return SCNGeometryElement(indices: indices, primitiveType: .triangleStrip)
    }

    var source: SCNGeometrySource {
        return SCNGeometrySource(vertices: vertices.map { SCNVector3($0) })
    }
}
