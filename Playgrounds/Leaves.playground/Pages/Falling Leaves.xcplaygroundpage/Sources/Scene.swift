import Foundation
import SceneKit
import simd
@testable import TreePhysics

public class Scene: SCNScene, SCNSceneRendererDelegate {
    let simulator: Simulator
    let emitter: Emitter
    
    public init(simulator: Simulator, emitter: Emitter) {
        self.simulator = simulator
        self.emitter = emitter
        super.init()
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    var i = 0
    
    public func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        emitter.update()
        simulator.update()
        if i % 5 == 0 {
            if let leaf = emitter.emit() {
                rootNode.addChildNode(leaf.node)
            }
        }
        i += 1
    }
    
    @objc public func airResistanceMultiplierSliderDidChange(sender: NSSlider) {
        print(sender.floatValue)
        simulator.airResistanceMultiplier = sender.floatValue
    }
    
    @objc public func phiSliderDidChange(sender: NSSlider) {
        print(sender.floatValue)
        simulator.phi = sender.floatValue
    }
}
