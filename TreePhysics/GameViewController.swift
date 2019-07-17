//
//  GameViewController.swift
//  TreePhysics
//
//  Created by Nick Kallen on 7/16/19.
//  Copyright © 2019 Nick Kallen. All rights reserved.
//

import SceneKit
import QuartzCore

class GameViewController: NSViewController {
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        let scene = SCNScene()
        
        // create and add a camera to the scene
        let cameraNode = SCNNode()
        cameraNode.camera = SCNCamera()
        scene.rootNode.addChildNode(cameraNode)
        cameraNode.position = SCNVector3(x: 0, y: 1, z: 5)
        cameraNode.name = "Camera"
        
        // create and add an ambient light to the scene
        let ambientLightNode = SCNNode()
        ambientLightNode.light = SCNLight()
        ambientLightNode.light!.type = .ambient
        ambientLightNode.name = "Ambient Light"
        scene.rootNode.addChildNode(ambientLightNode)
        
        scnView.scene = scene
        scnView.allowsCameraControl = true
        scnView.showsStatistics = true
        scnView.backgroundColor = NSColor.black
    }

    override func viewDidAppear() {
//        add the tree
        let root = Branch()
        let b1 = Branch()
        let b2 = Branch()

        root.add(b1)
        b1.add(b2)
        let tree = Tree(root)

        b2.apply(force: float2(0,1), at: 1)
        tree.root.updateComposite()

        scnView.scene!.rootNode.addChildNode(tree.root.node)
    }

    var scnView: SCNView {
        return self.view as! SCNView
    }
}
