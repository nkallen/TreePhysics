import Foundation
import simd

class Cylinder {
    let vertices: [float3]

    init(radius: Float, height: Float, radialSegmentCount: Int = 48, heightSegmentCount: Int = 1) {
        var vertices: [float3] = []
        let arcLength: Float = 2.0 * .pi / Float(radialSegmentCount)
        let halfHeight: Float = height / 2.0
//        for i in 0..<heightSegmentCount {
            for j in 0..<radialSegmentCount {
                let theta = arcLength*Float(j)
                let rCosTheta = radius*cos(theta)
                let rSinTheta = radius*sin(theta)
                vertices.append(float3(rCosTheta, -halfHeight, rSinTheta))
                vertices.append(float3(rCosTheta, halfHeight, rSinTheta))
            }
//        }
        self.vertices = vertices
    }
}
