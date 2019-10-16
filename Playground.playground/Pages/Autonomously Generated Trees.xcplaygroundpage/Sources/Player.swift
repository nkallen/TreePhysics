import Foundation
import Cocoa
import TreePhysics
import SceneKit

public protocol Playable: class {
    func update() -> SCNNode?
    func inspect() -> SCNNode?
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
    var previousNode: SCNNode? = nil

    public init(frame: CGRect) {
        super.init(nibName: nil, bundle: nil)

        let view = SCNView(frame: frame)
        self.view = view
        let scene = SCNScene()

        view.scene = scene
        view.backgroundColor = .gray
        view.showsStatistics = true
        view.allowsCameraControl = true

        let clickGesture = NSClickGestureRecognizer(target: self, action: #selector(handleClick(_:)))
        var gestureRecognizers = view.gestureRecognizers
        gestureRecognizers.insert(clickGesture, at: 0)
        view.gestureRecognizers = gestureRecognizers

        queue.async {
            while true {
                self.semaphore.wait()
                self.update()
                self.semaphore.signal()
            }
        }
    }

    private func update() {
        if let node = playable.update() {
            if let previousNode = previousNode {
                previousNode.removeFromParentNode()
            }
            let scene = scnView.scene!
            scene.rootNode.addChildNode(node)
            previousNode = node
        }
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }

    override public func keyDown(with event: NSEvent) {
        switch event.keyCode {
        case 0x31 where state == .playing:
            semaphore.wait()
            state = .paused
        case 0x31 where state == .paused:
            semaphore.signal()
            state = .playing
        case 0x22:
            if let previousNode = previousNode, let node = playable.inspect() {
                previousNode.addChildNode(node)
                print(2)
            }
        default: super.keyDown(with: event)
        }
    }

    override public func moveRight(_ sender: Any?) {
        switch state {
        case .paused:
            update()
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
