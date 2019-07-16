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
        
        // create and add an ambient light to the scene
        let ambientLightNode = SCNNode()
        ambientLightNode.light = SCNLight()
        ambientLightNode.light!.type = .ambient
        scene.rootNode.addChildNode(ambientLightNode)
        
        // add the tree
        let root = Branch()
        let b1 = Branch()
        let b2 = Branch()

        root.add(b1)
        b1.add(b2)
        let tree = Tree(root)

        b2.apply(force: float2(-1,0), at: 1)
        tree.root.updateComposite()
        for branch in [tree.root, b1, b2] {
            print(branch.name, branch.compositeMass, branch.compositeForce)
        }

        scene.rootNode.addChildNode(tree.root.node)

        let scnView = self.view as! SCNView
        scnView.scene = scene
        scnView.allowsCameraControl = true
        scnView.showsStatistics = true
        scnView.backgroundColor = NSColor.black
    }
}
