import Foundation
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
