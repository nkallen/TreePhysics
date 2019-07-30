import Foundation
import simd

// The interface of a "pen" which can draw a tree; concrete implementations include a cylinder
// pen (which makes mesh geometry) and a rigid body pen, which generates the rigid body.

protocol Pen {
    associatedtype T

    func start(at: float3, thickness: Float)
    func cont(distance: Float, tangent: float3, thickness: Float) -> T
    var branch: Self { get }
}
