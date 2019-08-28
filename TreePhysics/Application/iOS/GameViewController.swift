#if os(iOS)
import Foundation
import SceneKit
import UIKit

class GameViewController: UIViewController {
    var scene: Scene!

    override func viewDidLoad() {
        super.viewDidLoad()

        self.scene = Scene()

        scnView.scene = scene.scene
        scnView.allowsCameraControl = true
        scnView.showsStatistics = true
        scnView.backgroundColor = .black
        scnView.preferredFramesPerSecond = 60

        let tapGesture = UITapGestureRecognizer(target: self, action: #selector(handleTap(_:)))
        let panGesture = UIPanGestureRecognizer(target: self, action: #selector(handlePan(_:)))
        panGesture.minimumNumberOfTouches = 1
        panGesture.maximumNumberOfTouches = 1
        scnView.addGestureRecognizer(tapGesture)
        scnView.addGestureRecognizer(panGesture)
    }

    override func viewDidAppear(_ animated: Bool) {
        scnView.delegate = scene
    }

    var scnView: SCNView {
        return self.view as! SCNView
    }
}

fileprivate var toggle: Bool = false

extension GameViewController {
    @objc
    func handleTap(_ gestureRecognizer: UIGestureRecognizer) {
        toggle = !toggle
        print(scnView.hitTest(gestureRecognizer.location(in: scnView), options: nil))
        scene.gravityField.g = toggle ? float3.zero : float3(0, -9.81, 0)
    }

    @objc
    func handlePan(_ gestureRecognizer: UIGestureRecognizer) {
        let point = gestureRecognizer.location(in: scnView)

        let projectedOrigin = scnView.projectPoint(SCNVector3Zero)
        let vpWithZ = SCNVector3(x: Float(point.x), y: Float(point.y), z: projectedOrigin.z)
        let worldPoint = float3(scnView.unprojectPoint(vpWithZ))

        scene.attractorField.position = worldPoint
        scene.attractor.simdPosition = worldPoint
    }
}

#endif
