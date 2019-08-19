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

    BufferIndexDebugString = 10,

};

typedef NS_ENUM(NSInteger, ThreadGroupIndex)
{
    ThreadGroupIndexRigidBodies = 0,
    ThreadGroupIndexCompositeBodies  = 1,
};

typedef NS_ENUM(NSInteger, FunctionConstantIndex)
{
    FunctionConstantIndexRangeCount = 0,
};

typedef struct {
    vector_half3 position;
    half mass;
    matrix_half3x3 inertiaTensor;
    vector_half3 force;
    vector_half3 torque;
    vector_half3 centerOfMass;
} CompositeBodyStruct;

typedef struct {
    // const:
    int parentId;
    int childIds[3]; // Q: if we do level order, can we just do like climberOffset
    int climberOffset;
    ushort childCount;
    ushort climberCount;
    half mass;
    half length;
    half radius;
    matrix_half3x3 localRotation;

    vector_half3 position;
    matrix_half3x3 rotation;
    matrix_half3x3 inertiaTensor;
    vector_half3 centerOfMass;
    
    vector_half3 force;
    vector_half3 torque;
} RigidBodyStruct;

typedef struct {
    matrix_half3x3 θ;
    half k;
} JointStruct;

typedef struct {
    vector_half3 position;
    vector_half3 halfExtent;

} PhysicsFieldStruct;

#endif /* ShaderTypes_h */
