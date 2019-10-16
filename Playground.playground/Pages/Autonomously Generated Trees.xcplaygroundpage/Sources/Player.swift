import Foundation
import Cocoa
import TreePhysics
import SceneKit

public protocol Playable: class {
    func update()
}

public class PlayerViewController: NSViewController {
    let queue = DispatchQueue(label: #function)
    let semaphore = DispatchSemaphore(value: 0)

    enum State {
        case paused
        case playing
    }
    private var state: State = .paused

    public weak var playable: Playable!

    public func setup(view: SCNView) {
        self.view = view
        let clickGesture = NSClickGestureRecognizer(target: self, action: #selector(handleClick(_:)))
        var gestureRecognizers = scnView.gestureRecognizers
        gestureRecognizers.insert(clickGesture, at: 0)
        scnView.gestureRecognizers = gestureRecognizers

        queue.async {
            while true {
                self.semaphore.wait()
                self.playable.update()
                self.semaphore.signal()
            }
        }
    }

    override public func keyDown(with event: NSEvent) {
        switch event.keyCode {
        case 0x31 where state == .playing:
            semaphore.wait()
            state = .paused
        case 0x31 where state == .paused:
            semaphore.signal()
            state = .playing
        default: super.keyDown(with: event)
        }
    }

    override public func moveRight(_ sender: Any?) {
        switch state {
        case .paused:
            playable.update()
        case .playing: ()
        }
    }

    @objc
    func handleClick(_ gestureRecognizer: NSGestureRecognizer) {
        print(scnView.hitTest(gestureRecognizer.location(in: scnView), options: nil))
    }

    var scnView: SCNView {
        return self.view as! SCNView
    }
}
