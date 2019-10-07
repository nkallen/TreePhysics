import Foundation
import simd

// The interface of a "pen" which can draw a tree; concrete implementations include a cylinder
// pen (which makes mesh geometry) and a rigid body pen, which generates the rigid body.

public protocol Pen {
    associatedtype T

    func start(at: float3, orientation: simd_quatf, thickness: Float)
    func cont(distance: Float, orientation: simd_quatf, thickness: Float) -> T
    func copy(scale: Float, orientation: simd_quatf) -> T
    func branch() -> Self
}

