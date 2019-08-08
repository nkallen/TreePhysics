import Foundation
import simd

protocol HasTransform {
    var transform: matrix_float4x4 { get }
}

extension HasTransform {
    func convert(position: float3) -> float3 {
        return (transform * float4(position, 1)).xyz
    }

    var position: float3 {
        return convert(position: float3.zero) // XXX FIXME
    }

    var rotation: float3x3 {
        return matrix3x3_rotation(from: transform)
    }
}
