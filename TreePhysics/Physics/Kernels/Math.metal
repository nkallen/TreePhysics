#include <metal_stdlib>
#include "Math.h"
#include "ShaderTypes.h"

using namespace metal;

template <class T>
inline T
sqr(T A)
{
    return A * A;
}

template <class T>
inline matrix<T, 3, 3>
skew(vec<T, 3> v)
{
    return matrix<T, 3, 3>(
                           vec<T, 3>(0, v.z, -v.y),
                           vec<T, 3>(-v.z, 0, v.x),
                           vec<T, 3>(v.y, -v.x, 0));
}

template <class T>
inline matrix<T, 3, 3>
skew(packed_vec<T, 3> v)
{
    return matrix<T, 3, 3>(
                           vec<T, 3>(0, v.z, -v.y),
                           vec<T, 3>(-v.z, 0, v.x),
                           vec<T, 3>(v.y, -v.x, 0));
}

template <class T>
inline matrix<T, 3, 3>
cholesky(matrix<T, 3, 3> A)
{
    matrix<T, 3, 3> L = matrix<T, 3, 3>(0);

    // Load A into registers

    T a00 = A[0][0];
    T a01 = A[0][1], a11 = A[1][1];
    T a02 = A[0][2], a12 = A[1][2], a22 = A[2][2];

    // Factorize A
    T l00 = rsqrt(a00);
    T l01 = a01*l00;
    T l02 = a02*l00;

    T l11 = sqrt(a11 - sqr(l01));
    T l12 = (a12 - l02 * l01) / l11;

    T l22 = sqrt(a22 - sqr(l02) - sqr(l12));

    // Store L into memory
    L[0][0] = a00*l00;
    L[0][1] = l01; L[1][1] = l11;
    L[0][2] = l02; L[1][2] = l12; L[2][2] = l22;

    return L;
}

template <class T>
inline matrix<T, 3, 3>
inverse(matrix<T, 3, 3> m)
{
    T a00 = m[0][0], a01 = m[0][1], a02 = m[0][2];
    T a10 = m[1][0], a11 = m[1][1], a12 = m[1][2];
    T a20 = m[2][0], a21 = m[2][1], a22 = m[2][2];


    T b10 = a22 * a11 - a21 * a12;
    T b11 = -a22 * a01 + a21 * a02;
    T b12 = a12 * a01 - a11 * a02;

    T det = a00 * b10 + a10 * b11 + a20 * b12;

    return 1/det * matrix<T, 3, 3>(vec<T, 3>(b10, b11, b12),
                                   vec<T, 3>((-a22 * a10 + a20 * a12), (a22 * a00 - a20 * a02), (-a12 * a00 + a10 * a02)),
                                   vec<T, 3>((a21 * a10 - a20 * a11), (-a21 * a00 + a20 * a01), (a11 * a00 - a10 * a01)));
}

template <class T>
inline matrix<T, 3, 3>
inverse_lowerTriangular(matrix<T, 3, 3> m)
{
    T a00 = m[0][0], a01 = m[0][1], a02 = m[0][2];
    T                a11 = m[1][1], a12 = m[1][2];
    T                               a22 = m[2][2];


    T b10 = a22 * a11;
    T b11 = -a22 * a01;
    T b12 = a12 * a01 - a11 * a02;

    T det = a00 * b10;

    return 1/det * matrix<T, 3, 3>(vec<T, 3>(b10, b11, b12),
                                   vec<T, 3>(0, (a22 * a00), (-a12 * a00)),
                                   vec<T, 3>(0, 0, (a11 * a00)));
}

template <class T>
inline matrix<T, 2, 2>
inverse(matrix<T, 2, 2> m) {
    return ((T)1.0 / determinant(m)) *
    matrix<T, 2, 2>(vec<T, 2>(m[1][1],-m[0][1]),
                    vec<T, 2>(-m[1][0], m[0][0]));
}

template <class T>
inline matrix<T, 3, 3>
matrix_rotate(vec<T, 3> rotation)
{
    T pitch = rotation.x;
    T yaw = rotation.y;
    T roll = rotation.z;

    T cb = cos(pitch);
    T sb = sin(pitch);
    T ch = cos(yaw);
    T sh = sin(yaw);
    T ca = cos(roll);
    T sa = sin(roll);

    return matrix<T, 3, 3>(
                           vec<T, 3>(ch*ca,          sa,     -sh*ca),
                           vec<T, 3>(sh*sb-ch*sa*cb, ca*cb,  sh*sa*cb+ch*sb),
                           vec<T, 3>(ch*sa*sb+sh*cb, -ca*sb, -sh*sa*sb + ch*cb));
}

// MARK: - Differential Equations

enum QuadraticSolutionType
{
    QuadraticSolutionTypeReal,
    QuadraticSolutionTypeRealDistinct,
    QuadraticSolutionTypeComplex,
};

template <class T>
struct QuadraticSolution
{
    QuadraticSolutionType type;
    T a;
    T b;
};

template <class T>
struct DifferentialSolution {
    QuadraticSolutionType type;
    T c1;
    T c2;
    T x;
    T y;
    T z;
};

template <class T>
inline QuadraticSolution<T>
solveQuadratic(T a, T b, T c) {
    //    (-b +/- sqrt(b^2 - 4ac)) / 2a
    //    where r2 = c/ar1, cf: https://math.stackexchange.com/questions/311382/solving-a-quadratic-equation-with-precision-when-using-floating-point-variables
    T b2_4ac = b*b - 4.0*a*c;
    T _2a = 2.0*a;

    if (b2_4ac == 0) {
        return {
            QuadraticSolutionTypeReal,
            -b / _2a
        };
    } else if (b2_4ac > 0) {
        T r2 = (-b - sqrt(b2_4ac)) / (2.0*a);
        T r1 = c / (a * r2);
        return {
            QuadraticSolutionTypeRealDistinct,
            r1,
            r2
        };
    } else {
        T imaginaryPart = sqrt(-b2_4ac) / _2a;
        T realPart = -b / _2a;
        return {
            QuadraticSolutionTypeComplex,
            realPart,
            imaginaryPart
        };
    }
}

template <class T>
inline DifferentialSolution<T>
solveDifferential(T a, T b, T c, T g, T y_0, T y_ddt_0)
{
    T k = g/c;
    T y_0_k = y_0 - k;
    QuadraticSolution<T> quadraticSolution = solveQuadratic(a, b, c);
    switch (quadraticSolution.type) {
        case QuadraticSolutionTypeReal: {
            matrix<T, 2, 2> system = matrix<T, 2, 2>(vec<T, 2>(1, quadraticSolution.a), vec<T, 2>(0, 1));
            vec<T, 2> solution = inverse(system) * vec<T, 2>(y_0_k, y_ddt_0);
            return {
                QuadraticSolutionTypeReal,
                solution.x,
                solution.y,
                quadraticSolution.a,
                0,
                k
            };
        }
        case QuadraticSolutionTypeRealDistinct: {
            matrix<T, 2, 2> system = matrix<T, 2, 2>(vec<T, 2>(1, quadraticSolution.a), vec<T, 2>(1, quadraticSolution.b));
            vec<T, 2> solution = inverse(system) * vec<T, 2>(y_0_k, y_ddt_0);
            return {
                QuadraticSolutionTypeRealDistinct,
                solution.x,
                solution.y,
                quadraticSolution.a,
                quadraticSolution.b,
                k
            };
        }
        case QuadraticSolutionTypeComplex: {
            T c1 = y_0_k;
            T c2 = (y_ddt_0 - quadraticSolution.a * c1) / quadraticSolution.b;
            return {
                QuadraticSolutionTypeComplex,
                c1,
                c2,
                quadraticSolution.a,
                quadraticSolution.b,
                k
            };
        }
    }
}

// Evaluate 2nd-order differential equation given its analytic solution
template <class T>
inline vec<T, 3>
evaluateDifferential(DifferentialSolution<T> differentialSolution, T t)
{
    T e = M_E_F;
    T c1 = differentialSolution.c1;
    T c2 = differentialSolution.c2;
    T r1 = differentialSolution.x;
    T r2 = differentialSolution.y;
    T k = differentialSolution.z;

    T y, y_ddt, y_d2dt;
    T pe1 = pow(e,r1*t);

    switch (differentialSolution.type) {
        case QuadraticSolutionTypeComplex:
            y = c1*pe1*cos(r2*t) + c2*pe1*sin(r2*t) + k;
            y_ddt = r1*c1*pe1*cos(r2*t) - r2*c1*pe1*sin(r2*t) +
            r1*c2*pe1*sin(r2*t) + r2*c2*pe1*cos(r2*t);
            y_d2dt = r1*r1*c1*pe1*cos(r2*t) - r2*r1*c1*pe1*sin(r2*t) -
            (r1*r2*c1*pe1*sin(r2*t) + r2*r2*c1*pe1*cos(r2*t)) +
            r1*r1*c2*pe1*sin(r2*t) + r2*r1*c2*pe1*cos(r2*t) +
            r1*r2*c2*pe1*cos(r2*t) - r2*r2*c2*pe1*sin(r2*t);
            break;
        case QuadraticSolutionTypeReal:
            y = c1*pe1 + c2*t*pe1 + k;
            y_ddt = r1*c1*pe1 +
            c2*pe1 + r1*c2*t*pe1;
            y_d2dt = r1*r1*c1*pe1 +
            r1*c2*pe1 +
            r1*c2*pe1 + r1*r1*c2*t*pe1;
            break;
        case QuadraticSolutionTypeRealDistinct:
            y = c1*pe1 + c2*pow(e,r2*t) + k;
            y_ddt = r1*c1*pe1 + r2*c2*pow(e,r2*t);
            y_d2dt = r1*r1*c1 * pe1 + r2*r2*c2 * pow(e,r2*t);
            break;
    }
    return vec<T, 3>(y, y_ddt, y_d2dt);
}

template <class T>
inline vec<T, 3>
evaluateDifferential(T a, T b, T c, T g, T y_0, T y_ddt_0, T t)
{
    DifferentialSolution<T> differentialSolution = solveDifferential(a, b, c, g, y_0, y_ddt_0);
    return evaluateDifferential(differentialSolution, t);
}

// MARK: Quaternions

// Quaternion and diagonlization adapted from Apple's "ForwardPlusWithTileShading"

/// A single-precision quaternion type

inline quatf quaternion(float x, float y, float z, float w) {
    return (quatf){ x, y, z, w };
}

inline quatf quaternion(float3 v, float w) {
    return (quatf){ v.x, v.y, v.z, w };
}

inline quatf quat_identity() {
    return quaternion(0, 0, 0, 1);
}

inline quatf quat_from_axis_angle(float3 axis, float radians) {
    float t = radians * 0.5;
    return quatf(axis.x * sin(t), axis.y * sin(t), axis.z * sin(t), cos(t));
}

inline quatf quat_from_euler(float3 euler) {
    quatf q;

    float cx = cos(euler.x / 2.f);
    float cy = cos(euler.y / 2.f);
    float cz = cos(euler.z / 2.f);
    float sx = sin(euler.x / 2.f);
    float sy = sin(euler.y / 2.f);
    float sz = sin(euler.z / 2.f);

    q.w = cx * cy * cz + sx * sy * sz;
    q.x = sx * cy * cz - cx * sy * sz;
    q.y = cx * sy * cz + sx * cy * sz;
    q.z = cx * cy * sz - sx * sy * cz;

    return q;
}

inline quatf quaternion(float3x3 m) {
    float m00 = m.columns[0].x;
    float m11 = m.columns[1].y;
    float m22 = m.columns[2].z;
    float x = sqrt(1 + m00 - m11 - m22) * 0.5;
    float y = sqrt(1 - m00 + m11 - m22) * 0.5;
    float z = sqrt(1 - m00 - m11 + m22) * 0.5;
    float w = sqrt(1 + m00 + m11 + m22) * 0.5;
    return quaternion(x, y, z, w);
}

inline float quat_length(quatf q) {
    return sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
}

inline float quat_length_squared(quatf q) {
    return q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
}

inline quatf quat_normalize(quatf q) {
    return q / quat_length(q);
}

inline float3 quat_axis(quatf q) {
    // This query doesn't make sense if w > 1, but we do our best by
    // forcing q to be a unit quaternion if it obviously isn't
    if (q.w > 1.0)
    {
        q = quat_normalize(q);
    }

    float axisLen = sqrt(1 - q.w * q.w);

    if (axisLen < 1e-5)
    {
        // At lengths this small, direction is arbitrary
        return float3(1, 0, 0);
    }
    else
    {
        return float3(q.x / axisLen, q.y / axisLen, q.z / axisLen);
    }
}

inline float quat_angle(quatf q) {
    return 2 * acos(q.w);
}

inline quatf quat_conjugate(quatf q) {
    return quaternion(-q.x, -q.y, -q.z, q.w);
}

inline quatf quat_inverse(quatf q) {
    return quat_conjugate(q) / quat_length_squared(q);
}

inline quath quat_inverse(quath q) {
    return (quath)quat_inverse((quatf)q);
}

inline quatf quat_multiply(quatf q0, quatf q1) {
    quatf q;

    q.x = q0.w*q1.x + q0.x*q1.w + q0.y*q1.z - q0.z*q1.y;
    q.y = q0.w*q1.y - q0.x*q1.z + q0.y*q1.w + q0.z*q1.x;
    q.z = q0.w*q1.z + q0.x*q1.y - q0.y*q1.x + q0.z*q1.w;
    q.w = q0.w*q1.w - q0.x*q1.x - q0.y*q1.y - q0.z*q1.z;
    return q;
}

inline quatf quat_slerp(quatf q0, quatf q1, float t) {
    quatf q;

    float cosHalfTheta = dot(q0, q1);
    if (fabs(cosHalfTheta) >= 1.f) ///q0=q1 or q0=q1
    {
        return q0;
    }

    float halfTheta = acos(cosHalfTheta);
    float sinHalfTheta = sqrt(1.f - cosHalfTheta * cosHalfTheta);
    if (fabs(sinHalfTheta) < 0.001f)
    {    // q0 & q1 180 degrees not defined
        return q0*0.5f + q1*0.5f;
    }
    float srcWeight = sin((1 - t) * halfTheta) / sinHalfTheta;
    float dstWeight = sin(t * halfTheta) / sinHalfTheta;

    q = srcWeight*q0 + dstWeight*q1;

    return q;
}

inline float3 quat_act(quatf q, float3 v) {
    float3 qp = float3(q.x, q.y, q.z);
    float w = q.w;
    return 2 * dot(qp, v) * qp +
    ((w * w) - dot(qp, qp)) * v +
    2 * w * cross(qp, v);
}

inline float3x3 float3x3_from_quat(quatf q) {
    float xx = q.x * q.x;
    float xy = q.x * q.y;
    float xz = q.x * q.z;
    float xw = q.x * q.w;
    float yy = q.y * q.y;
    float yz = q.y * q.z;
    float yw = q.y * q.w;
    float zz = q.z * q.z;
    float zw = q.z * q.w;

    float m00 = 1 - 2 * (yy + zz);
    float m01 = 2 * (xy - zw);
    float m02 = 2 * (xz + yw);

    float m10 = 2 * (xy + zw);
    float m11 = 1 - 2 * (xx + zz);
    float m12 = 2 * (yz - xw);

    float m20 = 2 * (xz - yw);
    float m21 = 2 * (yz + xw);
    float m22 = 1 - 2 * (xx + yy);

    return float3x3(m00, m10, m20,
                    m01, m11, m21,
                    m02, m12, m22);
}

inline float3x3 float3x3_from_inertiaTensor(InertiaTensor t) {
    float3x3 result = float3x3(0);

    result[0][0] = (float)t.diag.x;
    result[1][1] = (float)t.diag.y;
    result[2][2] = (float)t.diag.z;

    result[0][1] = (float)t.ltr.x;
    result[0][2] = (float)t.ltr.y;
    result[1][2] = (float)t.ltr.z;

    result[1][0] = (float)t.ltr.x;
    result[2][0] = (float)t.ltr.y;
    result[2][1] = (float)t.ltr.z;

    return result;
}

inline InertiaTensor inertiaTensor_from_float3x3(float3x3 x) {
    packed_float3 diag = packed_float3(x[0][0], x[1][1], x[2][2]);
    packed_float3 ltr  = packed_float3(x[0][1], x[0][2], x[1][2]);
    return {.diag = diag, .ltr = ltr};
}

inline void
swap(thread uint &a, thread uint &b)
{
    uint tmp = a;
    a = b;
    b = tmp;
}
