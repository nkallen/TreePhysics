import Foundation
import ModelIO

extension MDLMesh {
    var vertices: [SIMD3<Float>] {
        let attributeData = self.vertexAttributeData(forAttributeNamed: "position")!
        precondition(attributeData.stride % MemoryLayout<Float>.stride == 0)

        let verticesPointer = attributeData.map.bytes.bindMemory(to: Float.self, capacity: attributeData.bufferSize)

        var result: [SIMD3<Float>] = []
        let stride = attributeData.stride / MemoryLayout<Float>.stride
        for i in 0..<self.vertexCount {
            let x = verticesPointer[i * stride + 0]
            let y = verticesPointer[i * stride + 1]
            let z = verticesPointer[i * stride + 2]

            result.append(SIMD3<Float>(x,y,z))
        }
        return result
    }
}
