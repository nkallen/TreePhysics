import Foundation
import ModelIO
import SceneKit.ModelIO

extension MDLMesh {
    var vertices: [simd_float3] {
        let attributeData = self.vertexAttributeData(forAttributeNamed: "position")!
        precondition(attributeData.stride % MemoryLayout<Float>.stride == 0)

        let verticesPointer = attributeData.map.bytes.bindMemory(to: Float.self, capacity: attributeData.bufferSize)

        var result: [simd_float3] = []
        let stride = attributeData.stride / MemoryLayout<Float>.stride
        for i in 0..<self.vertexCount {
            let x = verticesPointer[i * stride + 0]
            let y = verticesPointer[i * stride + 1]
            let z = verticesPointer[i * stride + 2]

            result.append(simd_float3(x,y,z))
        }
        return result
    }
}
