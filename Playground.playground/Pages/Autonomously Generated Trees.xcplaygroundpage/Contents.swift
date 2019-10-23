import AppKit
import SceneKit
import PlaygroundSupport
import ModelIO
import SceneKit.ModelIO
@testable import TreePhysics

var config = AutoTree.Config()
config.internodeLength = 0.03
config.occupationRadius = config.internodeLength * 1
config.perceptionRadius = config.internodeLength * 8
config.apicalDominance = 0.53
config.baseRadius = 0.03
config.extremityRadius = 0.005
config.sensitivityOfBudsToLight = 2.0

config.branchGravitropismBias = 0.05
config.branchStraightnessBias = 0.75

config.maxShootLength = 5
config.gravitropismAngle = 0 // FIXME make angle consistent (the other tropism is relative to UP not HEADING)
config.branchingAngle = .pi/5
config.phyllotacticAngle = 137 * .pi / 180  // vs .pi/2
config.fullExposure = 1
config.shadowDecayFactor = 0.75
config.shadowIntensity = 0.1
config.shadowDepth = 12
config.initialShadowGridSize = 256

config.horizontalGravimorphismBias = 1
config.verticalGravimorphismBias = 1
config.upperSideGravimorphismBias = 0

config.deflectionAngle = config.branchingAngle

config.leafRadiusThreshold = 0.01
config.petioleLength = config.internodeLength / 2
config.leafScale = config.internodeLength
config.eh = 0.2 // FIXME rename
config.petioleSegmentCount = 10

let autoTree = AutoTree(config)

let (root, _) = autoTree.seedling()
let shadowGrid = AutoTree.ArrayBackedShadowGrid(config)
let simulator = autoTree.growthSimulator(shadowGrid: shadowGrid)
simulator.addRoot(root)

var enableAllBuds = false

let url = Bundle.main.url(forResource: "ARFaceGeometry", withExtension: "obj")!
let asset = MDLAsset(url: url)
let mdlMesh = asset.object(at: 0) as! MDLMesh

let face = SCNNode(mdlObject: mdlMesh)
let scale: Float = config.internodeLength / 3
let offset = SIMD3<Float>(0,3.5,-0.5)
face.simdScale = SIMD3<Float>(repeating: 1) * scale
face.simdPosition = offset

let vertices: [SIMD3<Float>] = mdlMesh.vertices.map { $0 * scale + offset }
//simulator.addAttractionPoints(vertices)

var success = false

let material: SCNMaterial = {
    let url = Bundle.main.url(forResource: "bark", withExtension: "png")!
    let image = NSImage(byReferencing: url)

    let material = SCNMaterial()
    material.diffuse.contents = image
    material.diffuse.wrapS = .repeat
    material.diffuse.wrapT = .repeat
    return material
}()

extension AutoTree.GrowthSimulator: Playable {
    public func update() -> SCNNode? {
        do {
            if !success {
//                simulator.addAttractionPoints(vertices)
            }
            try simulator.update(enableAllBuds: false)
            success = true

            // FIXME radialSegmentCount
            let pen = CylinderPen<UInt32>(radialSegmentCount: 15, parent: nil, material: material)
            pen.start(at: root.position, orientation: root.orientation, thickness: config.baseRadius)
            autoTree.draw(root, pen: pen, showBuds: false)
            return pen.node()
        } catch {
            print(error)
            try! simulator.update(enableAllBuds: true)
            let pen = CylinderPen<UInt32>(radialSegmentCount: 15, parent: nil, material: material)
            pen.start(at: root.position, orientation: root.orientation, thickness: config.baseRadius)
            autoTree.draw(root, pen: pen, showBuds: false)
            return pen.node()
        }
        return nil
    }

    public func inspect() -> SCNNode? {
        let plot = Plot<UInt32>()
        plot.scatter(points: Array(simulator.attractionPoints), scale: config.internodeLength)
        //        plot.voxels(data: shadowGrid.storage, size: shadowGrid.size, scale: config.internodeLength)
        return plot.node()
        //        return draw(root)    }
    }
}

let viewController = PlayerViewController(frame: CGRect(x:0 , y:0, width: 640, height: 480))
//viewController.scnView.scene?.rootNode.addChildNode(face)
viewController.playable = simulator
PlaygroundPage.current.liveView = viewController
