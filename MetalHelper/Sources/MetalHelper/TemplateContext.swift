import Foundation
import MetalKit

@objcMembers public final class TemplateContext: NSObject {
    public let functions: Functions
    public let argument: [String: NSObject]

    public var function: [String: Function] {
        return functions.functionsByName
    }

    public init(functions: Functions, arguments: [String: NSObject]) {
        self.functions = functions
        self.argument = arguments
    }

    // FIXME is this necessary?
    public var stencilContext: [String: Any] {
        return [
            "functions": functions,
            "function": functions.functionsByName,
            "argument": argument
        ]
    }
}

extension ProcessInfo {
    public var context: TemplateContext! {
        return NSKeyedUnarchiver.unarchiveObject(withFile: arguments[1]) as? TemplateContext
    }
}

@objcMembers public final class Functions: NSObject {

    public let functions: [Function]

    public init(functions: [Function]) {
        self.functions = functions
    }

    public lazy internal(set) var functionsByName: [String: Function] = {
        var functionsByName = [String: Function]()
        self.functions.forEach { functionsByName[$0.name] = $0 }
        return functionsByName
    }()

    public lazy internal(set) var all: [Function] = {
        return self.functions
    }()

    public lazy internal(set) var vertexes: [Function] = {
        return self.functions.filter { $0.type == "vertex" }
    }()

    public lazy internal(set) var fragments: [Function] = {
        return self.functions.filter { $0.type == "fragment" }
    }()

    public lazy internal(set) var kernels: [Function] = {
        return self.functions.filter { $0.type == "kernel" }
    }()
}

@objcMembers public final class Function: NSObject {
    public var name: String
    public var type: String
//    public var vertexAttributes: [VertexAttribute]?
    public var stageInputAttributes: [Attribute]?
    public var constants: [FunctionConstant]

    init(name: String, type: String, constants: [FunctionConstant], stageInputAttributes: [Attribute]?) {
        self.name = name
        self.type = type
        self.constants = constants
        self.stageInputAttributes = stageInputAttributes
    }
}

@objcMembers public final class FunctionConstant: NSObject {
    var name: String
    var type: String
    var index: Int
    var required: Bool

    init(name: String, type: String, index: Int, required: Bool) {
        self.name = name
        self.type = type
        self.index = index
        self.required = required
    }
}

@objcMembers public final class Attribute: NSObject {
    var name: String
    var attributeIndex: Int
    var attributeType: String
    var isActive: Bool
    var isPatchControlPointData: Bool
    var isPatchData: Bool

    init(name: String, attributeIndex: Int, attributeType: String, isActive: Bool, isPatchControlPointData: Bool, isPatchData: Bool) {
        self.name = name
        self.attributeIndex = attributeIndex
        self.attributeType = attributeType
        self.isActive = isActive
        self.isPatchControlPointData = isPatchControlPointData
        self.isPatchData = isPatchData
    }
}

func Type(of t: MTLDataType) -> String {
    let result: String
    switch t {
    case .none:
        result = "none"
    case .struct:
        result = "struct"
    case .array:
        result = "array"
    case .float:
        result = "float"
    case .float2:
        result = "float2"
    case .float3:
        result = "float3"
    case .float4:
        result = "float4"
    case .float2x2:
        result = "float2x2"
    case .float2x3:
        result = "float2x3"
    case .float2x4:
        result = "float2x4"
    case .float3x2:
        result = "float3x2"
    case .float3x3:
        result = "float3x3"
    case .float3x4:
        result = "float3x4"
    case .float4x2:
        result = "float4x2"
    case .float4x3:
        result = "float4x3"
    case .float4x4:
        result = "float4x4"
    case .half:
        result = "half"
    case .half2:
        result = "half2"
    case .half3:
        result = "half3"
    case .half4:
        result = "half4"
    case .half2x2:
        result = "half2x2"
    case .half2x3:
        result = "half2x3"
    case .half2x4:
        result = "half2x4"
    case .half3x2:
        result = "half3x2"
    case .half3x3:
        result = "half3x3"
    case .half3x4:
        result = "half3x4"
    case .half4x2:
        result = "half4x2"
    case .half4x3:
        result = "half4x3"
    case .half4x4:
        result = "half4x4"
    case .int:
        result = "int"
    case .int2:
        result = "int2"
    case .int3:
        result = "int3"
    case .int4:
        result = "int4"
    case .uint:
        result = "uint"
    case .uint2:
        result = "uint2"
    case .uint3:
        result = "uint3"
    case .uint4:
        result = "uint4"
    case .short:
        result = "short"
    case .short2:
        result = "short2"
    case .short3:
        result = "short3"
    case .short4:
        result = "short4"
    case .ushort:
        result = "ushort"
    case .ushort2:
        result = "ushort2"
    case .ushort3:
        result = "ushort3"
    case .ushort4:
        result = "ushort4"
    case .char:
        result = "char"
    case .char2:
        result = "char2"
    case .char3:
        result = "char3"
    case .char4:
        result = "char4"
    case .uchar:
        result = "uchar"
    case .uchar2:
        result = "uchar2"
    case .uchar3:
        result = "uchar3"
    case .uchar4:
        result = "uchar4"
    case .bool:
        result = "bool"
    case .bool2:
        result = "bool2"
    case .bool3:
        result = "bool3"
    case .bool4:
        result = "bool4"
    case .texture:
        result = "texture"
    case .sampler:
        result = "sampler"
    case .pointer:
        result = "pointer"
    case .renderPipeline:
        result = "renderPipeline"
    case .indirectCommandBuffer:
        result = "indirectCommandBuffer"
    @unknown default:
        fatalError()
    }

    return result
}
