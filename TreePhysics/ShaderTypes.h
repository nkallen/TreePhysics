#ifndef ShaderTypes_h
#define ShaderTypes_h

#ifdef __METAL_VERSION__
#define NS_ENUM(_type, _name) enum _name : _type _name; enum _name : _type
#define NSInteger metal::int32_t
#else
#import <Foundation/Foundation.h>
#endif

#include <simd/simd.h>

typedef NS_ENUM(NSInteger, BufferIndex)
{
    BufferIndexRigidBodies = 0,
    BufferIndexCompositeBodies  = 1,
    BufferIndexGridOrigin = 2,
};

typedef NS_ENUM(NSInteger, ThreadGroupIndex)
{
    ThreadGroupIndexRigidBodies = 0,
    ThreadGroupIndexCompositeBodies  = 1,
    ThreadGroupIndexCompositeBodiesDone  = 2,
};

typedef struct {
    vector_float3 position;
    float mass;
    matrix_float3x3 inertiaTensor;
    vector_float3 force;
    vector_float3 torque;
    vector_float3 centerOfMass;
} CompositeBodyStruct;

typedef struct {
    int parentId;
    int childIds[5];
    uint childCount;
    vector_float3 position;
    float mass;
    float length;
    float radius;
    matrix_float3x3 inertiaTensor;

    vector_float3 force;
    vector_float3 torque;
    vector_float3 centerOfMass;
} RigidBodyStruct;

typedef struct {
    CompositeBodyStruct childCompositeBody;
    matrix_float3x3 worldToLocalRotation;
    matrix_float3x3 theta;
    float k;
} JointStruct;

#endif /* ShaderTypes_h */
