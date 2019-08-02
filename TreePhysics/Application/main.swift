import Foundation

#if os(macOS)
import Cocoa

let isRunningTests = NSClassFromString("XCTestCase") != nil

class TestingAppDelegate: NSResponder, NSApplicationDelegate {
    var window: NSWindow?
}

if !isRunningTests {
    _ = NSApplicationMain(CommandLine.argc, CommandLine.unsafeArgv)
} else {
    let appDelegate = TestingAppDelegate()
    let app = NSApplication.shared
    app.delegate = appDelegate
    NSApp.run()
}
#elseif os(iOS)
import UIKit
let appDelegate = AppDelegate()
let app = UIApplication.shared
app.delegate = appDelegate

UIApplicationMain(
    CommandLine.argc, CommandLine.unsafeArgv, nil, NSStringFromClass(AppDelegate.self)
)
#endif
