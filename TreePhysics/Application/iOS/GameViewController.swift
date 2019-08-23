#if os(iOS)
import Foundation
import SceneKit
import UIKit

class GameViewController: UIViewController {
    var game: Game!

    override func viewDidLoad() {
        super.viewDidLoad()

        self.game = Game()

        scnView.scene = game.scene
        scnView.allowsCameraControl = true
        scnView.showsStatistics = true
        scnView.backgroundColor = .black
        scnView.preferredFramesPerSecond = 10

        let tapGesture = UITapGestureRecognizer(target: self, action: #selector(handleTap(_:)))
        let panGesture = UIPanGestureRecognizer(target: self, action: #selector(handlePan(_:)))
        panGesture.minimumNumberOfTouches = 1
        panGesture.maximumNumberOfTouches = 1
        scnView.addGestureRecognizer(tapGesture)
        scnView.addGestureRecognizer(panGesture)
    }

    override func viewDidAppear(_ animated: Bool) {
        scnView.delegate = game
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
        game.gravityField.g = toggle ? float3.zero : float3(0, -9.81, 0)
    }

    @objc
    func handlePan(_ gestureRecognizer: UIGestureRecognizer) {
        let point = gestureRecognizer.location(in: scnView)

        let projectedOrigin = scnView.projectPoint(SCNVector3Zero)
        let vpWithZ = SCNVector3(x: Float(point.x), y: Float(point.y), z: projectedOrigin.z)
        let worldPoint = float3(scnView.unprojectPoint(vpWithZ))

        game.attractorField.position = worldPoint
        game.attractor.simdPosition = worldPoint
    }
}

#endif
