import Foundation
import SceneKit

// A composite pen that "draws" cylinders and rigid bodies simultaneously.

typealias Indices = [UInt16]

final class SkinningPen: Pen {
    typealias T = (Indices, RigidBody)

    let cylinderPen: CylinderPen
    let rigidBodyPen: RigidBodyPen
    weak var parent: SkinningPen?
    private(set) var bones: [T] = []

    init(cylinderPen: CylinderPen, rigidBodyPen: RigidBodyPen, parent: SkinningPen? = nil) {
        self.cylinderPen = cylinderPen
        self.rigidBodyPen = rigidBodyPen
        self.parent = parent
    }

    func start(at: float3, thickness: Float) {
        cylinderPen.start(at: at, thickness: thickness)
        rigidBodyPen.start(at: at, thickness: thickness)
    }

    func cont(distance: Float, tangent: float3, thickness: Float) -> T {
        let vertices = cylinderPen.cont(distance: distance, tangent: tangent, thickness: thickness)
        let rigidBody = rigidBodyPen.cont(distance: distance, tangent: tangent, thickness: thickness)
        return addBone((vertices, rigidBody))
    }

    func copy(scale: Float, orientation: simd_quatf) -> T {
        let vertices = cylinderPen.copy(scale: scale, orientation: orientation)
        let rigidBody = rigidBodyPen.copy(scale: scale, orientation: orientation)
        return addBone((vertices, rigidBody))
    }

    func addBone(_ bone: T) -> T {
        if let parent = parent {
            return parent.addBone(bone)
        } else {
            bones.append(bone)
            return bone
        }
    }

    var branch: SkinningPen {
        return SkinningPen(cylinderPen: cylinderPen.branch, rigidBodyPen: rigidBodyPen.branch, parent: self)
    }

    var node: SCNNode {
        var boneNodes: [SCNNode] = []
        var boneInverseBindTransforms: [NSValue] = []
        var boneWeights: [Float] = Array(repeating: 1.0, count: cylinderPen.branchGeometry.source.vectorCount)
        var boneIndices: Indices = Array(repeating: 0, count: cylinderPen.branchGeometry.source.vectorCount)

        for (boneIndex, bone) in bones.enumerated() {
            let (vertexIndices, rigidBody) = bone
            let node = rigidBody.node
            boneNodes.append(node)
            boneInverseBindTransforms.append(NSValue(scnMatrix4: SCNMatrix4Invert(node.worldTransform)))
            for vertexIndex in vertexIndices {
                boneIndices[Int(vertexIndex)] = UInt16(boneIndex)
            }
        }

        let boneWeightsData = Data(bytesNoCopy: &boneWeights, count: boneWeights.count * MemoryLayout<Float>.size, deallocator: .none)
        let boneIndicesData = Data(bytesNoCopy: &boneIndices, count: boneWeights.count * MemoryLayout<UInt16>.size, deallocator: .none)

        let boneWeightsGeometrySource = SCNGeometrySource(data: boneWeightsData, semantic: .boneWeights, vectorCount: boneWeights.count, usesFloatComponents: true, componentsPerVector: 1, bytesPerComponent: MemoryLayout<Float>.size, dataOffset: 0, dataStride: MemoryLayout<Float>.size)
        let boneIndicesGeometrySource = SCNGeometrySource(data: boneIndicesData, semantic: .boneIndices, vectorCount: boneIndices.count, usesFloatComponents: false, componentsPerVector: 1, bytesPerComponent: MemoryLayout<UInt16>.size, dataOffset: 0, dataStride: MemoryLayout<UInt16>.size)

        let skinner = SCNSkinner(baseGeometry: cylinderPen.branchGeometry.geometry, bones: boneNodes, boneInverseBindTransforms: boneInverseBindTransforms, boneWeights: boneWeightsGeometrySource, boneIndices: boneIndicesGeometrySource)

        let node = SCNNode(geometry: cylinderPen.branchGeometry.geometry)
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
