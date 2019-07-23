import Foundation
import simd

protocol Pen {
    func start(at: float2, thickness: Float)
    func cont(distance: Float, tangent: float2, thickness: Float)
    var branch: Pen { get }
}
