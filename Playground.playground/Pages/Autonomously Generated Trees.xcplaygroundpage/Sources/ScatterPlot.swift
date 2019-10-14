import Foundation
import TreePhysics
import simd
import SceneKit

public class ScatterPlot<I> where I: FixedWidthInteger {
    private let builder = GeometryBuilder<I>(primitiveType: .triangles)

    public init() {}

    public func add(points: [SIMD3<Float>], scale: Float = 1.0) {
        let vertices = [SIMD3<Float>(-0.5, 0, 0), SIMD3<Float>(0, 1, 0),
                        SIMD3<Float>(0, 1, 0), SIMD3<Float>(0.5, 0, 0)]
        let indices: [I] = [0,1,2, 0,2,3,
                            2,1,0, 3,2,0]

        for point in points {
            let verticesForPoint = vertices.map { point + scale * $0 }
            _ = builder.addSegment(verticesForPoint, indices)
        }
    }

    public func node() -> SCNNode {
        let points = SCNNode(geometry: builder.geometry)
        points.geometry?.firstMaterial!.diffuse.contents = NSColor(red: 1, green: 0.718, blue: 0.773, alpha: 0.9)
        return points
    }
}
