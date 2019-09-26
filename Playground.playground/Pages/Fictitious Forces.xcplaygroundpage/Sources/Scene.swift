import Foundation
import SceneKit
import simd
@testable import TreePhysics

public typealias UpdateAtCallback = (TimeInterval) -> ()

public class Scene: SCNScene, SCNSceneRendererDelegate {
    let delta: Float
    var callbacks: [UpdateAtCallback] = []

    public init(delta: Float = 1/60) {
        self.delta = delta
        super.init()
    }

    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }

    public func add(_ updateAtCallback: @escaping UpdateAtCallback) {
        callbacks.append(updateAtCallback)
    }

    public func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        for callback in callbacks {
            callback(time)
        }
    }
}
