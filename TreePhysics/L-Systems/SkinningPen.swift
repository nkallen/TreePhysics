import Foundation
import SceneKit

// A composite pen that "draws" cylinders and rigid bodies simultaneously.

public final class SkinningPen<I>: Pen where I: FixedWidthInteger {
    public typealias T = ([I], RigidBody)

    let branchBones: BoneBuilder<I>
    let leafBones: BoneBuilder<I>

    let cylinderPen: CylinderPen<I>
    let rigidBodyPen: RigidBodyPen
    weak var parent: SkinningPen?

    public init(cylinderPen: CylinderPen<I>, rigidBodyPen: RigidBodyPen, parent: SkinningPen? = nil) {
        self.cylinderPen = cylinderPen
        self.rigidBodyPen = rigidBodyPen
        self.parent = parent

        if let parent = parent {
            self.branchBones = parent.branchBones
            self.leafBones = parent.leafBones
        } else {
            self.branchBones = BoneBuilder()
            self.leafBones = BoneBuilder()
        }
    }

    public func start(at: float3, thickness: Float) {
        cylinderPen.start(at: at, thickness: thickness)
        rigidBodyPen.start(at: at, thickness: thickness)
    }

    public func cont(distance: Float, orientation: simd_quatf, thickness: Float) -> T {
        let vertices = cylinderPen.cont(distance: distance, orientation: orientation, thickness: thickness)
        let rigidBody = rigidBodyPen.cont(distance: distance, orientation: orientation, thickness: thickness)
        return branchBones.add(bone: (vertices, rigidBody))
    }

    public func copy(scale: Float, orientation: simd_quatf) -> T {
        let vertices = cylinderPen.copy(scale: scale, orientation: orientation)
        let rigidBody = rigidBodyPen.copy(scale: scale, orientation: orientation)
        return leafBones.add(bone: (vertices, rigidBody))
    }

    public var node: SCNNode {
        let parent = SCNNode()
        let branches = branchBones.node(for: cylinderPen.branchGeometry)
        let leaves = leafBones.node(for: cylinderPen.leafGeometry)
        parent.addChildNode(branches)
        parent.addChildNode(leaves)
        return parent
    }

    public var skeleton: SCNNode {
        let parent = SCNNode()
        let branches = branchBones.skeleton
        let leaves = leafBones.skeleton
        parent.addChildNode(branches)
        parent.addChildNode(leaves)
        return parent
    }

    public func branch() -> SkinningPen {
        return SkinningPen(cylinderPen: cylinderPen.branch(), rigidBodyPen: rigidBodyPen.branch(), parent: self)
    }
}

final class BoneBuilder<I> where I: FixedWidthInteger {
    typealias T = ([I], RigidBody)
    private(set) var bones: [T] = []

    func add(bone: T) -> T {
        bones.append(bone)
        return bone
    }

    func node(for builder: GeometryBuilder<I>) -> SCNNode {
        guard !bones.isEmpty else { return SCNNode() }

        var boneNodes: [SCNNode] = []
        var boneInverseBindTransforms: [NSValue] = []
        let vectorCount = builder.source.vectorCount
        var boneWeights: [Float] = Array(repeating: 1.0, count: vectorCount)
        var boneIndices: [I] = Array(repeating: 0, count: vectorCount)

        for (boneIndex, bone) in bones.enumerated() {
            let (vertexIndices, rigidBody) = bone
            let node = rigidBody.node
            boneNodes.append(node)
            boneInverseBindTransforms.append(NSValue(scnMatrix4: SCNMatrix4Invert(node.worldTransform)))
            for vertexIndex in vertexIndices {
                boneIndices[Int(vertexIndex)] = I(boneIndex)
            }
        }

        let boneWeightsData = Data(bytesNoCopy: &boneWeights, count: boneWeights.count * MemoryLayout<Float>.size, deallocator: .none)
        let boneIndicesData = Data(bytesNoCopy: &boneIndices, count: boneWeights.count * MemoryLayout<I>.size, deallocator: .none)

        let boneWeightsGeometrySource = SCNGeometrySource(data: boneWeightsData, semantic: .boneWeights, vectorCount: boneWeights.count, usesFloatComponents: true, componentsPerVector: 1, bytesPerComponent: MemoryLayout<Float>.size, dataOffset: 0, dataStride: MemoryLayout<Float>.size)
        let boneIndicesGeometrySource = SCNGeometrySource(data: boneIndicesData, semantic: .boneIndices, vectorCount: boneIndices.count, usesFloatComponents: false, componentsPerVector: 1, bytesPerComponent: MemoryLayout<I>.size, dataOffset: 0, dataStride: MemoryLayout<I>.size)

        let skinner = SCNSkinner(baseGeometry: builder.geometry, bones: boneNodes, boneInverseBindTransforms: boneInverseBindTransforms, boneWeights: boneWeightsGeometrySource, boneIndices: boneIndicesGeometrySource)

        let node = SCNNode(geometry: builder.geometry)
        node.skinner = skinner

        return node
    }

    var skeleton: SCNNode {
        let skeleton = SCNNode()

        for bone in bones {
            let (_, rigidBody) = bone
            let node = rigidBody.node
            skeleton.addChildNode(node)
        }

        return skeleton
    }
}
