import Foundation
import PathKit
import Commander
import Logging
import xcproj
import MetalKit
import SourceryFramework

class MetalHelper {
    public static let version: String = "Version"
    public static let generationMarker: String = "// Generated using MetalHelper"
    public static let generationHeader = "\(MetalHelper.generationMarker) \(MetalHelper.version)\n"
        + "// DO NOT EDIT\n\n"

    fileprivate let watcherEnabled: Bool
    fileprivate let arguments: [String: NSObject]

    fileprivate var status = ""
    fileprivate var templatesPaths = Paths(include: [])
    fileprivate var outputPath = Output("", linkTo: nil)
    fileprivate let prune: Bool

    // content annotated with file annotations per file path to write it to
    fileprivate var fileAnnotatedContent: [Path: [String]] = [:]

    fileprivate let device: MTLDevice

    init(device: MTLDevice, watcherEnabled: Bool = false, prune: Bool = false, arguments: [String: NSObject] = [:]) {
        self.device = device
        self.watcherEnabled = watcherEnabled
        self.arguments = arguments
        self.prune = prune
    }

    func processFiles(_ metallib: Path, usingTemplates templatesPaths: Paths, output: Output, forceParse: [String] = []) throws -> [FolderWatcher.Local]? {
        self.templatesPaths = templatesPaths
        self.outputPath = output

        let process: (Path) throws -> ParsingResult = { path in
            let result = try self.parse(from: path, forceParse: forceParse)

            try self.generate(templatePaths: templatesPaths, output: output, parsingResult: result)

            return result
        }

        var result = try process(metallib)

        guard watcherEnabled else {
            return nil
        }

        log.info("Starting watching sources.")

        let sourceWatchers = [
            FolderWatcher.Local(path: metallib.string) { events in
                let eventPaths: [Path] = events
                    .filter { $0.flags.contains(.isFile) }
                    .compactMap {
                        let path = Path($0.path)
                        return path.isMetalLibFile ? path : nil
                    }

                var pathThatForcedRegeneration: Path?
                for path in eventPaths {
                    guard let file = try? path.read(.utf8) else { continue }
                    if !file.hasPrefix(MetalHelper.generationMarker) {
                        pathThatForcedRegeneration = path
                        break
                    }
                }

                if let path = pathThatForcedRegeneration {
                    do {
                        log.info("Source changed at \(path.string)")
                        result = try process(path)
                    } catch {
                        log.error("\(error)")
                    }
                }
            }
        ]

        log.info("Starting watching templates.")

        let templateWatchers = topPaths(from: templatesPaths.allPaths).map({ templatesPath in
            return FolderWatcher.Local(path: templatesPath.string) { events in
                let events = events
                    .filter { $0.flags.contains(.isFile) && Path($0.path).isTemplateFile }

                if !events.isEmpty {
                    do {
                        if events.count == 1 {
                            log.info("Template changed \(events[0].path)")
                        } else {
                            log.info("Templates changed: ")
                        }
                        try self.generate(templatePaths: Paths(include: [templatesPath]), output: output, parsingResult: result)
                    } catch {
                        log.error("\(error)")
                    }
                }
            }
        })

        return Array([sourceWatchers, templateWatchers].joined())
    }

    private func topPaths(from paths: [Path]) -> [Path] {
        var top: [(Path, [Path])] = []
        paths.forEach { path in
            // See if its already contained by the topDirectories
            guard top.first(where: { (_, children) -> Bool in
                return children.contains(path)
            }) == nil else { return }

            if path.isDirectory {
                top.append((path, (try? path.recursiveChildren()) ?? []))
            } else {
                let dir = path.parent()
                let children = (try? dir.recursiveChildren()) ?? []
                if children.contains(path) {
                    top.append((dir, children))
                } else {
                    top.append((path, []))
                }
            }
        }

        return top.map { $0.0 }
    }

    fileprivate func templates(from: Paths) throws -> [Template] {
        return try templatePaths(from: from).compactMap {
            return try StencilTemplate(path: $0)
        }
    }

    private func templatePaths(from: Paths) -> [Path] {
        return from.allPaths.filter { $0.isTemplateFile }
    }
}

// MARK: - Parsing

extension MetalHelper {
    typealias ParsingResult = Functions

    fileprivate func parse(from filepath: Path, forceParse: [String] = []) throws -> ParsingResult {
        let library = try device.makeLibrary(filepath: filepath.string)
        var functions = [Function]()

        for functionName in library.functionNames {
            guard let mtlFunction = library.makeFunction(name: functionName) else {
                throw MTLLibraryError(.functionNotFound)
            }

            let constants: [FunctionConstant] = mtlFunction.functionConstantsDictionary.values.map {
                return FunctionConstant(
                    name: $0.name,
                    type: Type(of: $0.type),
                    index: $0.index,
                    required: $0.required)
            }

            let type: String
            switch mtlFunction.functionType {
            case .fragment:
                type = "fragment"
            case .kernel:
                type = "kernel"
            case .vertex:
                type = "vertex"
            @unknown default:
                fatalError()
            }

            let stageInputAttributes = mtlFunction.stageInputAttributes?.map {
                Attribute(
                    name: $0.name,
                    attributeIndex: $0.attributeIndex,
                    attributeType: Type(of: $0.attributeType),
                    isActive: $0.isActive,
                    isPatchControlPointData: $0.isPatchControlPointData,
                    isPatchData: $0.isPatchData)
            }

            functions.append(Function(name: functionName, type: type, constants: constants, stageInputAttributes: stageInputAttributes))
        }
        return Functions(functions: functions)
    }
}

// MARK: - Generation
extension MetalHelper {

    fileprivate func generate(templatePaths: Paths, output: Output, parsingResult: ParsingResult) throws {
        let generationStart = CFAbsoluteTimeGetCurrent()

        log.info("Loading templates...")
        let allTemplates = try templates(from: templatePaths)
        log.info("Loaded \(allTemplates.count) templates.")
        log.info("\tLoading took \(CFAbsoluteTimeGetCurrent() - generationStart)")

        log.info("Generating code...")
        status = ""

        if output.isDirectory {
            try allTemplates.forEach { template in
                let result = try generate(template, forParsingResult: parsingResult, outputPath: output.path)
                let outputPath = output.path + generatedPath(for: template.sourcePath)
                try self.output(result: result, to: outputPath)

                if let linkTo = output.linkTo {
                    link(outputPath, to: linkTo)
                }
            }
        } else {
            let result = try allTemplates.reduce("") { result, template in
                return result + "\n" + (try generate(template, forParsingResult: parsingResult, outputPath: output.path))
            }
            try self.output(result: result, to: output.path)

            if let linkTo = output.linkTo {
                link(output.path, to: linkTo)
            }
        }

        try fileAnnotatedContent.forEach { (path, contents) in
            try self.output(result: contents.joined(separator: "\n"), to: path)

            if let linkTo = output.linkTo {
                link(path, to: linkTo)
            }
        }

        if let linkTo = output.linkTo {
            try linkTo.project.writePBXProj(path: linkTo.projectPath)
        }

        log.info("\tGeneration took \(CFAbsoluteTimeGetCurrent() - generationStart)")
        log.info("Finished.")
    }

    private func link(_ output: Path, to linkTo: Output.LinkTo) {
        guard let target = linkTo.project.target(named: linkTo.target) else { return }

        let sourceRoot = linkTo.projectPath.parent()
        let fileGroup: PBXGroup
        if let group = linkTo.group {
            do {
                let addedGroup = linkTo.project.addGroup(named: group, to: linkTo.project.rootGroup, options: [])
                fileGroup = addedGroup.object
                if let groupPath = linkTo.project.fullPath(fileElement: addedGroup, sourceRoot: sourceRoot) {
                    try groupPath.mkpath()
                }
            } catch {
                log.warning("Failed to create a folder for group '\(fileGroup.name ?? "")'. \(error)")
            }
        } else {
            fileGroup = linkTo.project.rootGroup
        }
        do {
            try linkTo.project.addSourceFile(at: output, toGroup: fileGroup, target: target, sourceRoot: sourceRoot)
        } catch {
            log.warning("Failed to link file at \(output) to \(linkTo.projectPath). \(error)")
        }
    }

    private func output(result: String, to outputPath: Path) throws {
        var result = result
        if !result.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            if outputPath.extension == "swift" {
                result = MetalHelper.generationHeader + result
            }
            if !outputPath.parent().exists {
                try outputPath.parent().mkpath()
            }
            try writeIfChanged(result, to: outputPath)
        } else {
            if prune && outputPath.exists {
                log.info("Removing \(outputPath) as it is empty.")
                do { try outputPath.delete() } catch { log.error("\(error)") }
            } else {
                log.info("Skipping \(outputPath) as it is empty.")
            }
        }
    }

    private func generate(_ template: Template, forParsingResult parsingResult: ParsingResult, outputPath: Path) throws -> String {
        guard watcherEnabled else {
            let generationStart = CFAbsoluteTimeGetCurrent()
            let result = try Generator.generate(parsingResult, template: template, arguments: self.arguments)
            log.info("\tGenerating \(template.sourcePath.lastComponent) took \(CFAbsoluteTimeGetCurrent() - generationStart)")

            return try processRanges(in: parsingResult, result: result, outputPath: outputPath)
        }

        var result: String = ""
        do {
            result = try Generator.generate(parsingResult, template: template, arguments: self.arguments)
        } catch {
            log.error("\(error)")
            result = "\(error)"
        }

        return try processRanges(in: parsingResult, result: result, outputPath: outputPath)
    }

    private func processRanges(in parsingResult: ParsingResult, result: String, outputPath: Path) throws -> String {
        let start = CFAbsoluteTimeGetCurrent()
        defer {
            log.info("\t\tProcessing Ranges took \(CFAbsoluteTimeGetCurrent() - start)")
        }
        var result = result
        result = processFileRanges(for: parsingResult, in: result, outputPath: outputPath)
        return TemplateAnnotationsParser.removingEmptyAnnotations(from: result)
    }

    private func processFileRanges(`for` parsingResult: ParsingResult, in contents: String, outputPath: Path) -> String {
        let files = TemplateAnnotationsParser.parseAnnotations("file", contents: contents, aggregate: true)

        files
            .annotatedRanges
            .map { ($0, $1) }
            .forEach({ (filePath, ranges) in
                let generatedBody = ranges.map { contents.bridge().substring(with: $0.range) }.joined(separator: "\n")
                let path = outputPath + (Path(filePath).extension == nil ? "\(filePath).generated.swift" : filePath)
                var fileContents = fileAnnotatedContent[path] ?? []
                fileContents.append(generatedBody)
                fileAnnotatedContent[path] = fileContents
            })
        return files.contents
    }

    fileprivate func writeIfChanged(_ content: String, to path: Path) throws {
        guard path.exists else {
            return try path.write(content)
        }

        let existing = try path.read(.utf8)
        if existing != content {
            try path.write(content)
        }
    }

    private func indent(toInsert: String, indentation: String) -> String {
        guard indentation.isEmpty == false else {
            return toInsert
        }
        let lines = toInsert.components(separatedBy: "\n")
        return lines.enumerated()
            .map { index, line in
                guard !line.isEmpty else {
                    return line
                }

                return index == lines.count - 1 ? line : indentation + line
            }
            .joined(separator: "\n")
    }

    internal func generatedPath(`for` templatePath: Path) -> Path {
        return Path("\(templatePath.lastComponentWithoutExtension).generated.swift")
    }
}

public enum Generator {
    public static func generate(_ functions: Functions, template: Template, arguments: [String: NSObject] = [:]) throws -> String {
        log.info("Rendering template \(template.sourcePath)")
        return try template.render(TemplateContext(functions: functions, arguments: arguments))
    }
}
