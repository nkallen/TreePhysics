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

    BufferIndexDebug = 29,
    BufferIndexDebugLength = 30,
};

typedef NS_ENUM(NSInteger, ThreadGroupIndex)
{
    ThreadGroupIndexRigidBodies = 0,
    ThreadGroupIndexCompositeBodies  = 1,
};

typedef NS_ENUM(NSInteger, FunctionConstantIndex)
{
    FunctionConstantIndexRangeCount = 0,
    FunctionConstantIndexPhysicsFieldCount = 1,
};

typedef NS_ENUM(NSInteger, PhysicsFieldType)
{
    PhysicsFieldTypeGravity = 0,
    PhysicsFieldTypeWind = 1,
    PhysicsFieldTypeAttractor = 2,
};

typedef struct {
    packed_half3 g;
} GravityField;

typedef struct {
    packed_half3 windVelocity;
    half airResistanceMultiplier;
    half phi;
    half leafScale;
    half branchScale;
    half airDensity;
    half normal2tangentialDragCoefficientRatio;
} WindField;

typedef struct {
    packed_half3 position;
    packed_half3 halfExtent;
    union {
        GravityField gravity;
        WindField wind;
    };
    PhysicsFieldType type;

#ifdef __METAL_VERSION__
    bool appliesTo(half3 centerOfMass);
    metal::half2x3 apply(half mass);
#endif
} PhysicsField;

typedef struct {
    packed_float3 diag;
    packed_float3 ltr;
} InertiaTensor;

#endif /* ShaderTypes_h */
