import AppKit
import SceneKit
import PlaygroundSupport
import SceneKit.ModelIO
@testable import TreePhysics

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

let url = Bundle.main.url(forResource: "ARFaceGeometry", withExtension: "obj", subdirectory: "art.scnassets")!
let asset = MDLAsset(url: url)
let mdlMesh = asset.object(at: 0) as! MDLMesh


let face = SCNNode(mdlObject: mdlMesh)
let scale: Float = 0.01
let offset = SIMD3<Float>(0,1,-0.1)
face.simdScale = float3(repeating: 1) * scale
face.simdPosition = offset

let vertices = mdlMesh.vertices.map { $0 * scale + offset }
simulator.addAttractionPoints(vertices)


while true {
    print("iterating")
    do {
        try simulator.update()
    } catch AutoTree.Error.noAttractionPoints {
        print("No attraction points")
        for i in 0..<10 {
            try! simulator.update(enableAllBuds: true)
        }
        break   
    } catch AutoTree.Error.noSelectedBuds {
        print("No selected buds; Try fiddling with the config.perceptionRadius and config.perceptionAngle")
        break
    } catch AutoTree.Error.noVigor {
        print("Try fiddling with config.fullExposure and config.shadowIntensity")
        break
    }
}

let pen = CylinderPen<UInt32>(radialSegmentCount: 15, parent: nil)

autoTree.draw(root, pen: pen)

let view = SCNView(frame: CGRect(x:0 , y:0, width: 640, height: 480))
let scene = SCNScene()


scene.rootNode.addChildNode(pen.node())
//scene.rootNode.addChildNode(face)

view.scene = scene
view.backgroundColor = .gray
view.showsStatistics = true
view.allowsCameraControl = true
PlaygroundPage.current.liveView = view
