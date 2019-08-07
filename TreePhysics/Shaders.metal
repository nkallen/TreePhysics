#include <metal_stdlib>
using namespace metal;

#import "ShaderTypes.h"

// Ported from https://github.com/melax/sandbox/blob/3e267f2db2262a4cc6bf3f576c8c92b3cba79efc/include/geometric.h

float4 qmul(float4 a, float4 b) {
    return float4(
                  a.x*b.w+a.w*b.x+a.y*b.z-a.z*b.y,
                  a.y*b.w+a.w*b.y+a.z*b.x-a.x*b.z,
                  a.z*b.w+a.w*b.z+a.x*b.y-a.y*b.x,
                  a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z);
}

inline float3 qxdir(float4 q) {
    return {q.w*q.w+q.x*q.x-q.y*q.y-q.z*q.z, (q.x*q.y+q.z*q.w)*2, (q.z*q.x-q.y*q.w)*2};
}

inline float3 qydir (float4 q) {
    return {(q.x*q.y-q.z*q.w)*2, q.w*q.w-q.x*q.x+q.y*q.y-q.z*q.z, (q.y*q.z+q.x*q.w)*2};
}

inline float3 qzdir (float4 q) {
    return {(q.z*q.x+q.y*q.w)*2, (q.y*q.z-q.x*q.w)*2, q.w*q.w-q.x*q.x-q.y*q.y+q.z*q.z};
}

inline float3x3 qmat(float4 q) {
    return {qxdir(q), qydir(q), qzdir(q)};
}

inline float4 diagonalize(float3x3 A)
{
    // A must be a symmetric matrix.
    // returns orientation of the principle axes.
    // returns quaternion q such that its corresponding column major matrix Q
    // can be used to Diagonalize A
    // Diagonal matrix D = transpose(Q) * A * (Q);  thus  A == Q*D*QT
    // The directions of q (cols of Q) are the eigenvectors D's diagonal is the eigenvalues
    // As per 'col' convention if float3x3 Q = qgetmatrix(q); then Q*v = q*v*conj(q)
    int maxsteps = 24;  // certainly wont need that many.
    int i;
    float4 q(0, 0, 0, 1);
    for (i = 0; i<maxsteps; i++)
    {
        float3x3 Q = qmat(q); // Q*v == q*v*conj(q)
        float3x3 D = transpose(Q) * A * Q;  // A = Q*D*Q^T
        float3 offdiag(D[1][2], D[0][2], D[0][1]); // elements not on the diagonal
        float3 om(fabs(offdiag.x), fabs(offdiag.y), fabs(offdiag.z)); // mag of each offdiag elem
        int k = (om.x>om.y && om.x>om.z) ? 0 : (om.y>om.z) ? 1 : 2; // index of largest element of offdiag
        int k1 = (k + 1) % 3;
        int k2 = (k + 2) % 3;
        if (offdiag[k] == 0.0f) break;  // diagonal already
        float thet = (D[k2][k2] - D[k1][k1]) / (2.0f*offdiag[k]);
        float sgn = (thet>0.0f) ? 1.0f : -1.0f;
        thet *= sgn; // make it positive
        float t = sgn / (thet + ((thet<1.E6f) ? sqrt(thet*thet + 1.0f) : thet)); // sign(T)/(|T|+sqrt(T^2+1))
        float c = 1.0f / sqrt(t*t + 1.0f); //  c= 1/(t^2+1) , t=s/c
        if (c == 1.0f) break;  // no room for improvement - reached machine precision.
        float4 jr(0, 0, 0, 0); // jacobi rotation for this iteration.
        jr[k] = sgn*sqrt((1.0f - c) / 2.0f);  // using 1/2 angle identity sin(a/2) = sqrt((1-cos(a))/2)
        jr[k] *= -1.0f; // note we want a final result semantic that takes D to A, not A to D
        jr.w = sqrt(1.0f - (jr[k] * jr[k]));
        if (jr.w == 1.0f) break; // reached limits of floating point precision
        q = qmul(q, jr);
        q = normalize(q);
    }
    float h = 1.0f/sqrt(2.0f);  // M_SQRT2
    float3x3 M = transpose(qmat(q)) * A * qmat(q);
    float3 e = float3(M[0][0],M[1][1],M[2][2]);
    q = (e.x < e.z)  ? qmul(q, float4( 0, h, 0, h )) : q;
    q = (e.y < e.z)  ? qmul(q, float4( h, 0, 0, h )) : q;
    q = (e.x < e.y)  ? qmul(q, float4( 0, 0, h, h )) : q;   // size order z,y,x so xy spans a planeish spread
    q = (qzdir(q).z < 0) ? qmul(q, float4( 1, 0, 0, 0 )) : q;
    q = (qydir(q).y < 0) ? qmul(q, float4( 0, 0, 1, 0 )) : q;
    q = (q.w < 0) ? -q : q;
    return q;
}

kernel void diagonalize(
                        constant float3x3 & A [[ buffer(0) ]],
                        device float3 *out [[ buffer(1) ]],
                        uint2 gid [[ thread_position_in_grid ]])
{
    float4 q = diagonalize(A);
    float3x3 M = transpose(qmat(q)) * A * qmat(q);
    out[gid.y] = float3(M[0][0], M[1][1], M[2][2]);
}

inline float3 jointRotateVector(
                                const device JointStruct & joint,
                                const float3 vector)
{
    return joint.worldToLocalRotation * vector;
}

inline float3x3 sqr(float3x3 A) {
    return A * A;
}

inline float3x3 crossMatrix(float3 v) {
    return float3x3(
                    float3(0, v.z, -v.y),
                    float3(-v.z, 0, v.x),
                    float3(-v.y, -v.x, 0));
}

inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody,
                              const CompositeBodyStruct childCompositeBodies[5])
{
    CompositeBodyStruct compositeBody;

    compositeBody.mass = rigidBody.mass;
    compositeBody.force = rigidBody.force;
    compositeBody.torque = rigidBody.torque;
    compositeBody.centerOfMass = rigidBody.mass * rigidBody.centerOfMass;
    compositeBody.position = rigidBody.position;

    for (size_t i = 0; i < rigidBody.childCount; i++) {
        CompositeBodyStruct childCompositeBody = childCompositeBodies[i];

        compositeBody.mass += childCompositeBody.mass;
        compositeBody.force += childCompositeBody.force;
        compositeBody.torque += cross(childCompositeBody.position - rigidBody.position, childCompositeBody.force) + childCompositeBody.torque;
        compositeBody.centerOfMass += childCompositeBody.mass * childCompositeBody.centerOfMass;
    }
    compositeBody.centerOfMass /= compositeBody.mass;

    compositeBody.inertiaTensor = rigidBody.inertiaTensor - rigidBody.mass * sqr(crossMatrix(rigidBody.centerOfMass - compositeBody.centerOfMass));

    for (size_t i = 0; i < rigidBody.childCount; i++) {
        CompositeBodyStruct childCompositeBody = childCompositeBodies[i];

        compositeBody.inertiaTensor += childCompositeBody.inertiaTensor - childCompositeBody.mass * sqr(crossMatrix(childCompositeBody.centerOfMass - compositeBody.centerOfMass));
    }

    return compositeBody;
}

inline void
rigidBody_childRigidBodies(
                           RigidBodyStruct rigidBody,
                           device RigidBodyStruct * rigidBodies,
                           RigidBodyStruct childRigidBodies[5])
{
    for (ushort i = 0; i < rigidBody.childCount; i++) {
        int childId = rigidBody.childIds[i];
        childRigidBodies[i] = childRigidBodies[childId];
    }
}


inline void
rigidBody_childCompositeBodies(
                               const RigidBodyStruct rigidBody,
                               device CompositeBodyStruct * compositeBodies,
                               CompositeBodyStruct childCompositeBodies[5])
{
    for (ushort i = 0; i < rigidBody.childCount; i++) {
        int childId = rigidBody.childIds[i];
        childCompositeBodies[i] = compositeBodies[childId];
    }
}

inline uint
rigidBody_childCompositeBodies(
                               const RigidBodyStruct rigidBody,
                               device CompositeBodyStruct * compositeBodies,
                               uint minIdInThreadGroup,
                               CompositeBodyStruct childCompositeBodies[5])
{
    uint missing = 0;
    for (ushort i = 0; i < rigidBody.childCount; i++) {
        uint childId = rigidBody.childIds[i];
        if (childId >= minIdInThreadGroup) {
            missing++;
        } else {
            childCompositeBodies[i] = compositeBodies[childId];
        }
    }
    return missing;
}

inline void
rigidBody_childCompositeBodies(
                               const RigidBodyStruct rigidBody,
                               device CompositeBodyStruct * compositeBodies,
                               ushort minIdInThreadGroup,
                               threadgroup CompositeBodyStruct * compositeBodiesCache,
                               CompositeBodyStruct childCompositeBodies[5])
{
    for (ushort i = 0; i < rigidBody.childCount; i++) {
        int childId = rigidBody.childIds[i];
        if (childId >= minIdInThreadGroup) {
            ushort lid = childId - minIdInThreadGroup;
            childCompositeBodies[i] = compositeBodiesCache[lid];
        } else {
            childCompositeBodies[i] = compositeBodies[childId];
        }
    }
}

inline CompositeBodyStruct
rigidBody_updateCompositeBody(
                              const RigidBodyStruct rigidBody,
                              device RigidBodyStruct * rigidBodies,
                              device CompositeBodyStruct * compositeBodies)
{
    CompositeBodyStruct childCompositeBodies[5];
    rigidBody_childCompositeBodies(rigidBody, compositeBodies, childCompositeBodies);
    return rigidBody_updateCompositeBody(rigidBody, childCompositeBodies);
}

void
rigidBody_climb(const RigidBodyStruct rigidBody,
                const CompositeBodyStruct compositeBody,
                device RigidBodyStruct * rigidBodies,
                device CompositeBodyStruct * compositeBodies)
{
    RigidBodyStruct currentRigidBody = rigidBody;
    CompositeBodyStruct currentCompositeBody = compositeBody;

    while (currentRigidBody.parentId != -1) {
        int parentId = currentRigidBody.parentId;
        currentRigidBody = rigidBodies[parentId];
        if (currentRigidBody.childCount == 1) {
            CompositeBodyStruct childCompositeBodies[5] = {currentCompositeBody};
            currentCompositeBody = rigidBody_updateCompositeBody(currentRigidBody, childCompositeBodies);
            compositeBodies[parentId] = currentCompositeBody;
        } else {
            return;
        }
    }
}

kernel void
updateCompositeBodies(
                      device RigidBodyStruct * rigidBodies [[ buffer(BufferIndexRigidBodies) ]],
                      device CompositeBodyStruct * compositeBodies [[ buffer(BufferIndexCompositeBodies) ]],
                      constant uint * gridOrigin [[ buffer(BufferIndexGridOrigin) ]],
                      uint gid [[ thread_position_in_grid ]])
{
    int id = *gridOrigin + gid;
    RigidBodyStruct rigidBody = rigidBodies[id];
    CompositeBodyStruct compositeBody = rigidBody_updateCompositeBody(rigidBody, rigidBodies, compositeBodies);
    compositeBodies[id] = compositeBody;

    rigidBody_climb(rigidBody, compositeBody, rigidBodies, compositeBodies);
}

kernel void
updateCompositeBodies2(
                       device RigidBodyStruct * rigidBodies [[ buffer(BufferIndexRigidBodies) ]],
                       device CompositeBodyStruct * compositeBodies [[ buffer(BufferIndexCompositeBodies) ]],
                       threadgroup CompositeBodyStruct * compositeBodiesCache [[ threadgroup(ThreadGroupIndexCompositeBodies) ]],
                       threadgroup RigidBodyStruct * rigidBodiesCache [[ threadgroup(ThreadGroupIndexRigidBodies) ]],
                       constant uint * gridOrigin [[ buffer(BufferIndexGridOrigin) ]],
                       uint gid [[ thread_position_in_grid ]],
                       ushort lid [[thread_position_in_threadgroup]],
                       ushort lsize [[threads_per_threadgroup]],
                       uint gsize [[threads_per_grid]])
{
    threadgroup atomic_uint compositeBodiesDoneByThreadGroup;

    int id = *gridOrigin + gid;
    int minIdInThreadGroup = id - lid;

    RigidBodyStruct rigidBody = rigidBodies[id];

    CompositeBodyStruct childCompositeBodies[5];
    ushort missing = rigidBody_childCompositeBodies(rigidBody, compositeBodies, minIdInThreadGroup, childCompositeBodies);

    if (missing == 0) {
        CompositeBodyStruct compositeBody = rigidBody_updateCompositeBody(rigidBody, childCompositeBodies);
        compositeBodies[id] = compositeBody;
        atomic_fetch_or_explicit(&compositeBodiesDoneByThreadGroup, 1 << lid, memory_order_relaxed);
        rigidBody_climb(rigidBody, compositeBody, rigidBodies, compositeBodies);

        if (rigidBody.parentId >= minIdInThreadGroup) {
            compositeBodiesCache[lid] = compositeBody;
        }
    } else {
        atomic_fetch_and_explicit(&compositeBodiesDoneByThreadGroup, ~0 ^ (1 << lid), memory_order_relaxed);
        rigidBodiesCache[lid] = rigidBody;
    }

    threadgroup_barrier(mem_flags::mem_threadgroup);
    thread uint compositeBodiesDoneThread = atomic_load_explicit(&compositeBodiesDoneByThreadGroup, memory_order_relaxed);

    if (lid == 0) {
        for (ushort lid = 0; lid < lsize; lid++) {
            id = *gridOrigin + minIdInThreadGroup + lid;
            if (!(compositeBodiesDoneThread & (1 << lid))) {
                RigidBodyStruct rigidBody = rigidBodiesCache[lid];
                CompositeBodyStruct childCompositeBodies[5];
                rigidBody_childCompositeBodies(rigidBody, compositeBodies, minIdInThreadGroup, compositeBodiesCache, childCompositeBodies);
                CompositeBodyStruct compositeBody = rigidBody_updateCompositeBody(rigidBody, childCompositeBodies);
                compositeBodies[id] = compositeBody;
                rigidBody_climb(rigidBody, compositeBody, rigidBodies, compositeBodies);

                if (rigidBody.parentId >= minIdInThreadGroup) {
                    compositeBodiesCache[lid] = compositeBody;
                }
            }
        }
    }
}

