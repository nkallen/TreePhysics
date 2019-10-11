import Foundation
import TreePhysics
import os.signpost

let log = OSLog(
    subsystem: "com.nk",
    category: "TreePhysics"
)

let signpostID = OSSignpostID(log: log)

var config = AutoTree.Config()
config.internodeLength = 0.03
config.occupationRadius = 0.03 * 1
config.perceptionRadius = 0.03 * 9
config.biasVigorTowardsMainAxis = 0.5
config.baseRadius = 0.03
config.extremityRadius = 0.001
config.sensitivityOfBudsToLight = 1.1
let autoTree = AutoTree(config)

let (root, _) = autoTree.seedling()
let simulator = autoTree.growthSimulator()
simulator.addRoot(root)

while true {
    os_signpost(.begin, log: log, name: "Thing", signpostID: signpostID)
    print("iterating")
    do {
        try simulator.update(enableAllBuds: true)
    } catch AutoTree.Error.noAttractionPoints {
        print("No attraction points")
        break
    } catch AutoTree.Error.noSelectedBuds {
        print("No selected buds; Try fiddling with the config.perceptionRadius and config.perceptionAngle")
        break
    } catch AutoTree.Error.noVigor {
        print("Try fiddling with config.fullExposure and config.shadowIntensity")
        break
    }
    os_signpost(.end, log: log, name: "Thing", signpostID: signpostID)
}

let pen = CylinderPen<UInt32>(radialSegmentCount: 15, parent: nil)

autoTree.draw(root, pen: pen)
