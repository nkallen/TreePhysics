import AppKit
import SceneKit
import PlaygroundSupport
import SceneKit.ModelIO
@testable import TreePhysics

var config = AutoTree.Config()
config.internodeLength = 0.1
config.occupationRadius = config.internodeLength * 1
config.perceptionRadius = config.internodeLength * 5
config.biasVigorTowardsMainAxis = 0.52 // apical dominance
config.baseRadius = 0.1
config.extremityRadius = 0.01
config.sensitivityOfBudsToLight = 1.3 // bud light sensitivity
config.branchGravitropismBias = 2/10
config.branchStraightnessBias = 4/10
config.maxShootLength = 5
config.gravitropismAngle = 0
config.branchingAngle = .pi/4
config.phyllotacticAngle = .pi/4
config.fullExposure = 300
config.shadowDecayFactor = 0.1
config.shadowIntensity = 0.1
config.shadowDepth = 3
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
//simulator.addAttractionPoints(vertices)

let view = SCNView(frame: CGRect(x:0 , y:0, width: 640, height: 480))
let scene = SCNScene()

var previousNode: SCNNode? = nil
var enableAllBuds = false


view.scene = scene
view.backgroundColor = .gray
view.showsStatistics = true
view.allowsCameraControl = true
PlaygroundPage.current.liveView = view

let queue = DispatchQueue(label: "test")
queue.async {
    var i = 0
    while i < 55 {
        print("iterating")

        do {
            let buds = try simulator.update(enableAllBuds: enableAllBuds)
            let pen = CylinderPen<UInt32>(radialSegmentCount: 15, parent: nil)
            autoTree.draw(root, pen: pen, showBuds: false)
            let node = pen.node()
//
//                        for bud in buds {
//                            let sphere = SCNNode(geometry: SCNSphere(radius: 0.01))
//                            sphere.simdPosition = bud.position
//                            sphere.geometry?.firstMaterial?.diffuse.contents = NSColor.red
//                            node.addChildNode(sphere)
//                        }
//                        for point in simulator.attractionPoints {
//                            let box = SCNBox(width: 0.01, height: 0.01, length: 0.01, chamferRadius: 0)
//                            box.firstMaterial?.diffuse.contents = NSColor.blue
//                            let sphere = SCNNode(geometry: box)
//                            sphere.simdPosition = point
//                            node.addChildNode(sphere)
//                        }
//            print(simulator.attractionPoints.count)
            let plot = ScatterPlot<UInt16>()
            plot.add(points: Array(simulator.attractionPoints), scale: config.internodeLength)
            node.addChildNode(plot.node())
            if let previousNode = previousNode {
                previousNode.removeFromParentNode()
            }
            scene.rootNode.addChildNode(node)
            previousNode = node
        } catch AutoTree.Error.noAttractionPoints {
            print("No attraction points")
            enableAllBuds = true
        } catch AutoTree.Error.noSelectedBuds {
            print("No selected buds; Try fiddling with the config.perceptionRadius and config.perceptionAngle")
            break
        } catch AutoTree.Error.noVigor {
            print("Try fiddling with config.fullExposure and config.shadowIntensity")
            break
        } catch {
            print("Error", error)
            break
        }
        i += 1
    }

}


//scene.rootNode.addChildNode(face)

