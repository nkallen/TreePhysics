#ifndef ShaderTypes_h
#define ShaderTypes_h

#ifdef __METAL_VERSION__
#define NS_ENUM(_type, _name) enum _name : _type _name; enum _name : _type
#define NSInteger metal::int32_t

#else
#import <Foundation/Foundation.h>
#import "Half.h"
#endif

#include <simd/simd.h>

typedef NS_ENUM(NSInteger, BufferIndex)
{
    BufferIndexRigidBodies = 0,
    BufferIndexCompositeBodies  = 1,
    BufferIndexGridOrigin = 2,
    BufferIndexJoints = 3,
    BufferIndexTime = 4,
    BufferIndexRanges = 5,
    BufferIndexPhysicsField = 6, 

    BufferIndexUpdateCompositeBodiesIn = 10,
    BufferIndexUpdateCompositeBodiesOut = 11,

    BufferIndexDebug = 20,
    BufferIndexDebugLength = 21,
};

typedef NS_ENUM(NSInteger, ThreadGroupIndex)
{
    ThreadGroupIndexRigidBodies = 0,
    ThreadGroupIndexCompositeBodies  = 1,
};

typedef NS_ENUM(NSInteger, FunctionConstantIndex)
{
    FunctionConstantIndexRangeCount = 0,
    FunctionConstantIndexBeta = 1,
    FunctionConstantIndexPrintMaxLen = 10,
};

typedef struct {
    vector_float3 pivot;
    float mass;
    matrix_float3x3 inertiaTensor;
    vector_float3 force;
    vector_float3 torque;
    vector_float3 centerOfMass;
} CompositeBodyStruct;

typedef struct {
    // const:
    int parentId;
    int childIds[3]; // Q: if we do level order, can we just do like climberOffset
    int climberOffset;
    ushort childCount;
    ushort climberCount;

    float mass;
    matrix_float3x3 localInertiaTensor;

    float jointStiffness;
    matrix_float3x3 jointLocalRotation;

    vector_float3 pivot;
    vector_float3 localPivot;
    matrix_float3x3 rotation;
    matrix_float3x3 inertiaTensor;
    vector_float3 centerOfMass;

    vector_float3 force;
    vector_float3 torque;
} RigidBodyStruct;

typedef struct {
    matrix_float3x3 θ;
} JointStruct;

typedef struct {
    vector_float3 position;
    vector_float3 halfExtent;
} PhysicsFieldStruct;

typedef struct {
    packed_float3 diag;
    packed_float3 ltr;
} InertiaTensor;

#endif /* ShaderTypes_h */
