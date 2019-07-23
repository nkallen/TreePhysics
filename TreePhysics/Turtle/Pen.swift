import Foundation
import simd

protocol Pen {
    associatedtype T

    func start(at: float2, thickness: Float)
    func cont(distance: Float, tangent: float2, thickness: Float) -> T
    var branch: Self { get }
}
