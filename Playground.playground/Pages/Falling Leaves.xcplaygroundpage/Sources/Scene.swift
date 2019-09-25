import Foundation
import SceneKit
import simd
@testable import TreePhysics

let delta: Float = 1/60

public class Scene: SCNScene, SCNSceneRendererDelegate {
    let simulator: CPUSimulator
    let emitter: Emitter
    let windField: WindField
    
    public init(simulator: CPUSimulator, emitter: Emitter, windField: WindField) {
        self.simulator = simulator
        self.emitter = emitter
        self.windField = windField
        super.init()
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    var i = 0
    
    public func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        emitter.update()
        simulator.update(at: Double(delta))
        if i % 5 == 0 {
            if let leaf = emitter.emit() {
                rootNode.addChildNode(leaf.node)
            }
        }
        i += 1
    }
    
    @objc public func airResistanceMultiplierSliderDidChange(sender: NSSlider) {
        print(sender.floatValue)
        windField.airResistanceMultiplier = sender.floatValue
    }
    
    @objc public func phiSliderDidChange(sender: NSSlider) {
        print(sender.floatValue)
        windField.phi = sender.floatValue
    }
}
