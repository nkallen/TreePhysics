// swift-tools-version:5.1

import PackageDescription

let package = Package(
    name: "MetalHelper",
    platforms: [
        .macOS(.v10_13),
    ],
    products: [
        .executable(name: "metalhelper", targets: ["MetalHelper"]),
        .library(
            name: "MetalHelper",
            targets: ["MetalHelper"]),
    ],
    dependencies: [
        .package(url: "https://github.com/kylef/Commander.git", .exact("0.7.1")),
        .package(url: "https://github.com/krzysztofzablocki/Sourcery.git", .exact("0.17.0")),
        .package(url: "https://github.com/kylef/PathKit.git", .exact("0.9.2")),
        .package(url: "https://github.com/SwiftGen/StencilSwiftKit.git", .exact("2.7.0")),
        .package(url: "https://github.com/tuist/xcodeproj", .exact("4.3.1")),
        .package(url: "https://github.com/apple/swift-log.git", from: "1.0.0"),
    ],
    targets: [
        // Targets are the basic building blocks of a package. A target can define a module or a test suite.
        // Targets can depend on other targets in this package, and on products in packages which this package depends on.
        .target(
            name: "MetalHelper",
            dependencies: ["PathKit", "Commander", "Logging", "StencilSwiftKit", "xcproj", "SourceryFramework", "SourceryRuntime"]),
        .testTarget(
            name: "MetalHelperTests",
            dependencies: ["MetalHelper"]),
    ]
)
