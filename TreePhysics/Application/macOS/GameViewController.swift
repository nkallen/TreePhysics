#if os(macOS)
import Foundation
import SceneKit
@testable import TreePhysics

class GameViewController: NSViewController {
    var scene: Scene!

    override func viewDidLoad() {
        super.viewDidLoad()
        
        self.scene = Scene()

        scnView.scene = scene.scene
        scnView.allowsCameraControl = true
        scnView.showsStatistics = true
        scnView.backgroundColor = .black

        let clickGesture = NSClickGestureRecognizer(target: self, action: #selector(handleClick(_:)))
        var gestureRecognizers = scnView.gestureRecognizers
        gestureRecognizers.insert(clickGesture, at: 0)
        scnView.gestureRecognizers = gestureRecognizers
    }

    override func viewDidAppear() {
        scnView.delegate = scene
        scnView.mySceneViewDelegate = self
    }

    var scnView: MySceneView {
        return self.view as! MySceneView
    }
}

fileprivate var toggle: Bool = false

extension GameViewController {
    @objc
    func handleClick(_ gestureRecognizer: NSGestureRecognizer) {
        toggle = !toggle
        print(scnView.hitTest(gestureRecognizer.location(in: scnView), options: nil))
        scene.gravityField.g = toggle ? float3.zero : float3(0, -9.81, 0)
    }

}

protocol MySceneViewDelegate: class {
    func mouseMoved(with event: NSEvent, in view: SCNView)
}

extension GameViewController: MySceneViewDelegate {
    func mouseMoved(with event: NSEvent, in view: SCNView) {
        let nsPoint = event.locationInWindow

        let projectedOrigin = view.projectPoint(SCNVector3Zero)
        let vpWithZ = SCNVector3(x: nsPoint.x, y: nsPoint.y, z: projectedOrigin.z)
        let worldPoint = float3(view.unprojectPoint(vpWithZ))

        scene.attractorField.position = worldPoint
        scene.attractor.simdPosition = worldPoint
    }
}

class MySceneView: SCNView {
    weak var mySceneViewDelegate: MySceneViewDelegate!
    var trackingArea : NSTrackingArea?

    override func updateTrackingAreas() {
        if trackingArea != nil {
            self.removeTrackingArea(trackingArea!)
        }
        let options : NSTrackingArea.Options =
            [.mouseEnteredAndExited, .mouseMoved, .activeInKeyWindow]
        trackingArea = NSTrackingArea(rect: self.bounds, options: options,
                                      owner: self, userInfo: nil)
        self.addTrackingArea(trackingArea!)
    }

    override func mouseMoved(with event: NSEvent) {
        mySceneViewDelegate.mouseMoved(with: event, in: self)
    }
}
#endif
