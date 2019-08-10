#include <metal_stdlib>
using namespace metal;

#define KERNEL_DEBUG_FORMAL_PARAMETERS device RigidBodyStruct * debugRigidBodies [[ buffer(BufferIndexDebugRigidBody) ]], device CompositeBodyStruct * debugCompositeBodies [[ buffer(BufferIndexDebugCompositeBody) ]], device float * debugFloats [[ buffer(BufferIndexDebugFloat) ]], device float3 * debugFloat3s [[ buffer(BufferIndexDebugFloat3) ]], device float3x3 * debugFloat3x3s [[ buffer(BufferIndexDebugFloat3x3) ]]
#define FUNCTION_DEBUG_FORMAL_PARAMETERS device RigidBodyStruct * debugRigidBodies, device CompositeBodyStruct * debugCompositeBodies, device float * debugFloats, device float3 * debugFloat3s, device float3x3 * debugFloat3x3s
#define DEBUG_PARAMETERS debugRigidBodies, debugCompositeBodies, debugFloats, debugFloat3s, debugFloat3x3s
