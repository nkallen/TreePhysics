import Foundation
import simd

protocol HasTransform {
    var transform: matrix_float3x3 { get }
}

extension HasTransform {
    func convert(position: float2) -> float2 {
        return (transform * float3(position, 1)).xy
    }

    var position: float2 {
        return convert(position: float2.zero)
    }
}
