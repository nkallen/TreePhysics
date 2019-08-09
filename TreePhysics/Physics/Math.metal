#include <metal_stdlib>
using namespace metal;

// MARK: Quaternion

// Quaternion and diagonalization ported from https://github.com/melax/sandbox/blob/3e267f2db2262a4cc6bf3f576c8c92b3cba79efc/include/geometric.h

inline float4
qmul(float4 a, float4 b) {
    return float4(
                  a.x*b.w+a.w*b.x+a.y*b.z-a.z*b.y,
                  a.y*b.w+a.w*b.y+a.z*b.x-a.x*b.z,
                  a.z*b.w+a.w*b.z+a.x*b.y-a.y*b.x,
                  a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z);
}

inline float3
qxdir(float4 q) {
    return {q.w*q.w+q.x*q.x-q.y*q.y-q.z*q.z, (q.x*q.y+q.z*q.w)*2, (q.z*q.x-q.y*q.w)*2};
}

inline float3
qydir (float4 q) {
    return {(q.x*q.y-q.z*q.w)*2, q.w*q.w-q.x*q.x+q.y*q.y-q.z*q.z, (q.y*q.z+q.x*q.w)*2};
}

inline float3
qzdir (float4 q) {
    return {(q.z*q.x+q.y*q.w)*2, (q.y*q.z-q.x*q.w)*2, q.w*q.w-q.x*q.x-q.y*q.y+q.z*q.z};
}

inline float3x3
qmat(float4 q) {
    return {qxdir(q), qydir(q), qzdir(q)};
}

// MARK: Diagonlization

inline float4
diagonalize(float3x3 A)
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
        float thet;
        if (om.x>om.y && om.x>om.z) {
            thet = (D[2][2] - D[1][1]) / (2.0f*offdiag[0]);
        } else if (om.y>om.z) {
            thet = (D[0][0] - D[2][2]) / (2.0f*offdiag[1]);
        } else {
            thet = (D[1][1] - D[0][0]) / (2.0f*offdiag[2]);
        }
        if (offdiag[k] == 0.0f) break;  // diagonal already
        float sgn = sign(thet);
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

// MARK: General

inline float
sqr(float a) {
    return a * a;
}

inline float3x3
sqr(float3x3 A)
{
    return A * A;
}

inline half3x3
sqr(half3x3 A)
{
    return A * A;
}

inline
float3x3 crossMatrix(float3 v)
{
    return float3x3(
                    float3(0, v.z, -v.y),
                    float3(-v.z, 0, v.x),
                    float3(v.y, -v.x, 0));
}


inline half3x3
crossMatrix(half3 v)
{
    return half3x3(
                   half3(0, v.z, -v.y),
                   half3(-v.z, 0, v.x),
                   half3(-v.y, -v.x, 0));
}

inline float3x3
cholesky(float3x3 A)
{
    float3x3 L = float3x3(0);

    // Load A into registers

    float a00 = A[0][0];
    float a01 = A[0][1], a11 = A[1][1];
    float a02 = A[0][2], a12 = A[1][2], a22 = A[2][2];

    // Factorize A
    float l00 = sqrt(a00);
    float l01 = a01/l00;
    float l02 = a02/l00;

    float l11 = sqrt(a11 - sqr(l01));
    float l12 = (a12 - l02 * l01) / l11;

    float l22 = sqrt(a22 - sqr(l02) - sqr(l12));

    // Store L into memory
    L[0][0] = l00;
    L[0][1] = l01; L[1][1] = l11;
    L[0][2] = l02; L[1][2] = l12; L[2][2] = l22;

    return L;
}

inline float3x3
inverse(float3x3 m)
{
    float a00 = m[0][0], a01 = m[0][1], a02 = m[0][2];
    float a10 = m[1][0], a11 = m[1][1], a12 = m[1][2];
    float a20 = m[2][0], a21 = m[2][1], a22 = m[2][2];


    float b10 = a22 * a11 - a21 * a12;
    float b11 = -a22 * a01 + a21 * a02;
    float b12 = a12 * a01 - a11 * a02;

    float det = a00 * b10 + a10 * b11 + a20 * b12;

    return 1/det * float3x3(float3(b10, b11, b12),
                            float3((-a22 * a10 + a20 * a12), (a22 * a00 - a20 * a02), (-a12 * a00 + a10 * a02)),
                            float3((a21 * a10 - a20 * a11), (-a21 * a00 + a20 * a01), (a11 * a00 - a10 * a01)));
}

inline float2x2
inverse(float2x2 m) {
    return float2x2(float2(m[1][1],-m[0][1]),
                    float2(-m[1][0], m[0][0]) / (m[0][0]*m[1][1] - m[1][0]*m[0][1]));
}

inline float3x3 matrix_rotate(float3 rotation)
{
    float pitch = rotation.x;
    float yaw = rotation.y;
    float roll = rotation.z;

    float cb = cos(pitch);
    float sb = sin(pitch);
    float ch = cos(yaw);
    float sh = sin(yaw);
    float ca = cos(roll);
    float sa = sin(roll);

    return float3x3(
                    float3(ch*ca,          sa,     -sh*ca),
                    float3(sh*sb-ch*sa*cb, ca*cb,  sh*sa*cb+ch*sb),
                    float3(ch*sa*sb+sh*cb, -ca*sb, -sh*sa*sb + ch*cb));
}

// MARK: - Differential Equations

enum QuadraticSolutionType
{
    QuadraticSolutionTypeReal,
    QuadraticSolutionTypeRealDistinct,
    QuadraticSolutionTypeComplex,
};

struct QuadraticSolution
{
    QuadraticSolutionType type;
    float a;
    float b;
};

struct DifferentialSolution {
    QuadraticSolutionType type;
    float c1;
    float c2;
    float x;
    float y;
    float z;
};

inline QuadraticSolution
solveQuadratic(float a, float b, float c) {
    //    (-b +/- sqrt(b^2 - 4ac)) / 2a
    //    where r2 = c/ar1, cf: https://math.stackexchange.com/questions/311382/solving-a-quadratic-equation-with-precision-when-using-floating-point-variables
    float b2_4ac = b*b - 4.0*a*c;
    float _2a = 2.0*a;

    if (b2_4ac == 0) {
        return {
            QuadraticSolutionTypeReal,
            -b / _2a
        };
    } else if (b2_4ac > 0) {
        float r2 = (-b - sqrt(b2_4ac)) / (2.0*a);
        float r1 = c / (a * r2);
        return {
            QuadraticSolutionTypeRealDistinct,
            r1,
            r2
        };
    } else {
        float imaginaryPart = sqrt(-b2_4ac) / _2a;
        float realPart = -b / _2a;
        return {
            QuadraticSolutionTypeComplex,
            realPart,
            imaginaryPart
        };
    }
}

inline DifferentialSolution
solveDifferential(float a, float b, float c, float g, float y_0, float y_ddt_0)
{
    QuadraticSolution quadraticSolution = solveQuadratic(a, b, c);
    switch (quadraticSolution.type) {
        case QuadraticSolutionTypeReal: {
            float2x2 system = float2x2(float2(1, quadraticSolution.a), float2(0, 1));
            float2 solution = inverse(system) * float2(y_0, y_ddt_0);
            return {
                QuadraticSolutionTypeReal,
                solution.x,
                solution.y,
                quadraticSolution.a,
                g/c
            };
        }
        case QuadraticSolutionTypeRealDistinct: {
            float2x2 system = float2x2(float2(1, quadraticSolution.a), float2(1, quadraticSolution.b));
            float2 solution = inverse(system) * float2(y_0, y_ddt_0);
            return {
                QuadraticSolutionTypeRealDistinct,
                solution.x,
                solution.y,
                quadraticSolution.a,
                quadraticSolution.b,
                g/c
            };
        }
        case QuadraticSolutionTypeComplex: {
            float c1 = y_0;
            float c2 = (y_ddt_0 - quadraticSolution.a * c1) / quadraticSolution.b;
            return {
                QuadraticSolutionTypeComplex,
                c1,
                c2,
                quadraticSolution.a,
                quadraticSolution.b,
                g/c
            };
        }
    }
}

// Evaluate 2nd-order differential equation given its analytic solution
inline float3
evaluateDifferential(DifferentialSolution differentialSolution, float t)
{
    switch (differentialSolution.type) {
        case QuadraticSolutionTypeComplex: {
            float c1 = differentialSolution.c1;
            float c2 = differentialSolution.c2;
            float λ = differentialSolution.x;
            float μ = differentialSolution.y;
            float k = differentialSolution.z;

            float y = c1*pow(M_E_F,λ*t)*cos(μ*t) + c2*pow(M_E_F,λ*t)*sin(μ*t) + k;
            float y_ddt = λ*c1*pow(M_E_F,λ*t)*cos(μ*t) - μ*c1*pow(M_E_F,λ*t)*sin(μ*t) +
            λ*c2*pow(M_E_F,λ*t)*sin(μ*t) + μ*c2*pow(M_E_F,λ*t)*cos(μ*t);
            float y_d2dt = λ*λ*c1*pow(M_E_F,λ*t)*cos(μ*t) - μ*λ*c1*pow(M_E_F,λ*t)*sin(μ*t) -
            (λ*μ*c1*pow(M_E_F,λ*t)*sin(μ*t) + μ*μ*c1*pow(M_E_F,λ*t)*cos(μ*t)) +
            λ*λ*c2*pow(M_E_F,λ*t)*sin(μ*t) + μ*λ*c2*pow(M_E_F,λ*t)*cos(μ*t) +
            λ*μ*c2*pow(M_E_F,λ*t)*cos(μ*t) - μ*μ*c2*pow(M_E_F,λ*t)*sin(μ*t);
            return float3(y, y_ddt, y_d2dt);
        }
        case QuadraticSolutionTypeReal: {
            float c1 = differentialSolution.c1;
            float c2 = differentialSolution.c2;
            float r = differentialSolution.x;
            float k = differentialSolution.y;

            float y = c1*pow(M_E_F,r*t) + c2*t*pow(M_E_F,r*t) + k;
            float y_ddt = r*c1*pow(M_E_F,r*t) +
            c2*pow(M_E_F,r*t) + r*c2*t*pow(M_E_F,r*t);
            float y_d2dt = r*r*c1*pow(M_E_F,r*t) +
            r*c2*pow(M_E_F,r*t) +
            r*c2*pow(M_E_F,r*t) + r*r*c2*t*pow(M_E_F,r*t);
            return float3(y, y_ddt, y_d2dt);
        }
        case QuadraticSolutionTypeRealDistinct: {
            float c1 = differentialSolution.c1;
            float c2 = differentialSolution.c2;
            float r1 = differentialSolution.x;
            float r2 = differentialSolution.y;
            float k = differentialSolution.z;

            float y = c1*pow(M_E_F,r1*t) + c2*pow(M_E_F,r2*t) + k;
            float y_ddt = r1*c1*pow(M_E_F,r1*t) + r2*c2*pow(M_E_F,r2*t);
            float y_d2dt = r1*r1*c1 * pow(M_E_F,r1*t) + r2*r2*c2 * pow(M_E_F,r2*t);
            return float3(y, y_ddt, y_d2dt);
        }
    }
}

inline float3
evaluateDifferential(float a, float b, float c, float g, float y_0, float y_ddt_0, float t)
{
    DifferentialSolution differentialSolution = solveDifferential(a, b, c, g, y_0, y_ddt_0);
    return evaluateDifferential(differentialSolution, t);
}
