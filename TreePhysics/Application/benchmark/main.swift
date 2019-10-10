import Foundation
import TreePhysics

var config = AutoTree.Config()
config.internodeLength = 0.03
config.occupationRadius = 0.03 * 1
config.perceptionRadius = 0.03 * 6
config.biasVigorTowardsMainAxis = 0.5
config.baseRadius = 0.01
config.extremityRadius = 0.001
config.sensitivityOfBudsToLight = 1
let autoTree = AutoTree(config)

let (root, _) = autoTree.seedling()
let simulator = autoTree.growthSimulator()
simulator.addRoot(root)

for _ in 0...200 {
    print("iterating")
    do {
        try simulator.update(enableAllBuds: true)
    } catch AutoTree.Error.noAttractionPoints {
    } catch AutoTree.Error.noSelectedBuds {
    } catch AutoTree.Error.noVigor {
    }
}
