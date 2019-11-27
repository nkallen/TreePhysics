import Foundation
import SceneKit
import UIKit

class GameViewController: UIViewController {
    var root: ArticulatedRigidBody!
    var world: PhysicsWorld!
    
    var captureScope: MTLCaptureScope!
    var commandQueue: MTLCommandQueue!
    var captureManager: MTLCaptureManager!
    
    var device: MTLDevice!
    var mem: MemoryLayoutManager!
    var simulator: MetalSimulator!
    var cpuSimulator: CPUSimulator!
    
    override func viewDidAppear(_ animated: Bool) {
        self.root = ArticulatedRigidBody.static()
        let rigidBodyPen = RigidBodyPen(parent: root)
        let cylinderPen = CylinderPen<UInt16>(radialSegmentCount: 3)
        let skinningPen = SkinningPen(cylinderPen: cylinderPen, rigidBodyPen: rigidBodyPen)
        
        let rule = Rewriter.Rule(symbol: "A", replacement: #"[!"&FFFFFFFA&&J]/////[!"&FFFFFFFA&&J]/////[!"&FFFFFFFA&&J]"#)
        let lSystem = Rewriter.rewrite(premise: "A", rules: [rule], generations: 4)
        let configuration = InterpreterConfig(
            //            randomScale: 0.4,
            angle: .pi / 8,
            thickness: 0.002*0.002*Float.pi,
//            thicknessScale: 0.7,
            stepSize: 0.4,
            stepSizeScale: 0.8)
        let interpreter = Interpreter(configuration: configuration, pen: skinningPen)
        interpreter.interpret(lSystem)
        
        self.device = MTLCreateSystemDefaultDevice()!
        self.commandQueue = device.makeCommandQueue()!
        let gravity = GravityField(simd_float3(0,-0.1,0))
        let wind = WindField(windVelocity: simd_float3(1,0.1,1)*5)
        self.world = PhysicsWorld()
        world.add(field: wind)
        self.world.add(rigidBody: root)
        self.mem = MemoryLayoutManager(device: device, root: root, fields: [wind])
        self.simulator = MetalSimulator(device: device, mem: mem)
        self.cpuSimulator = CPUSimulator(world: world)
        
        scnView.delegate = self
        let scene = SCNScene()
        scene.rootNode.addChildNode(createAxesNode(quiverLength: 1, quiverThickness: 0.1))
        scnView.scene = scene
        scnView.allowsCameraControl = true
        scnView.backgroundColor = .gray
        
        
        scnView.scene?.rootNode.addChildNode(skinningPen.node())
        //        scnView.scene?.rootNode.addChildNode(skinningPen.skeleton)
        
        //        triggerProgrammaticCapture()
    }
    
    func triggerProgrammaticCapture() {
        let captureManager = MTLCaptureManager.shared()
        let captureDescriptor = MTLCaptureDescriptor()
        captureDescriptor.captureObject = self.device
        do {
            try captureManager.startCapture(with: captureDescriptor)
        } catch {
            fatalError("error when trying to capture: \(error)")
        }
    }
    
    var scnView: SCNView {
        return self.view as! SCNView
    }
}

extension GameViewController: SCNSceneRendererDelegate {
    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        let commandQueue = device.makeCommandQueue()!
        let commandBuffer = commandQueue.makeCommandBuffer()!
        
        simulator.encode(commandBuffer: commandBuffer, at: 1.0/60)
        commandBuffer.commit()
        commandBuffer.waitUntilCompleted()
        //        check()
        //        cpuSimulator.update(at: 1.0/60)
        
        SCNTransaction.begin()
        for rigidBody in self.world.rigidBodiesUnordered {
            let id = self.mem.rigidBodies.index[rigidBody]!
            rigidBody.node.simdPosition = simd_float3(self.mem.rigidBodies.centerOfMass[id])
            rigidBody.node.simdOrientation = simd_quatf(self.mem.rigidBodies.orientation[id])
        }
        SCNTransaction.commit()
        
        print(String.localizedStringWithFormat("%.2f ms", (commandBuffer.gpuEndTime - commandBuffer.gpuStartTime) * 1000), mem.rigidBodies.ranges)
    }
}

extension GameViewController {
    func check() {
        let cs = CPUSimulator(world: self.world)
        cs.updateCompositeBodies()
        
        self.mem.assertValid(otherwise: { id, error in
            let captureManager = MTLCaptureManager.shared()
            print(self.mem.rigidBodies.ranges)
            print(error)
            for range in mem.rigidBodies.ranges {
                if range.contains(id) {
                    print("Error in range: ", range, id - range.lowerBound)
                }
            }
            
            
            trace(id)
            //            for (i, r) in world.rigidBodiesLevelOrder.enumerated() {
            //                print(i, r.composite.inertiaTensor)
            //                print(i, float3x3(mem.joints.inertiaTensor[i]))
            //            }
            //            fatalError()
            DispatchQueue.main.sync {
                captureManager.stopCapture()
            }
        })
    }
    
    func trace(_ id: Int, indent: String = "") {
        let body = world.rigidBodiesLevelOrder[id]
        for child in body.children {
            let i = mem.rigidBodies.index[child]!
            print(indent, i, world.rigidBodiesLevelOrder[i].composite.inertiaTensor)
            print(indent, i, float3x3(mem.joints.inertiaTensor[i]))
            trace(i, indent: indent + "  ")
            print("====")
        }
    }
    
    func inspect(_ id: Int) {
        for range in self.mem.rigidBodies.ranges {
            if range.contains(id) {
                self.simulator.debug?.print(id - range.lowerBound)
            }
        }
    }
}
