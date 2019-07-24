import Foundation
import simd

protocol Pen {
    associatedtype T

    func start(at: float3, thickness: Float)
    func cont(distance: Float, tangent: float3, thickness: Float) -> T
    var branch: Self { get }
}
