#include <metal_stdlib>
using namespace metal;

// MARK: Quaternion

// Quaternion and diagonalization ported from https://github.com/melax/sandbox/blob/3e267f2db2262a4cc6bf3f576c8c92b3cba79efc/include/geometric.h

template <class T>
inline vec<T, 4>
qmul(vec<T, 4> a, vec<T, 4> b) {
    return vec<T, 4>(
                     a.x*b.w+a.w*b.x+a.y*b.z-a.z*b.y,
                     a.y*b.w+a.w*b.y+a.z*b.x-a.x*b.z,
                     a.z*b.w+a.w*b.z+a.x*b.y-a.y*b.x,
                     a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z);
}

template <class T>
inline vec<T, 3>
qxdir(vec<T, 4> q) {
    return {q.w*q.w+q.x*q.x-q.y*q.y-q.z*q.z, (q.x*q.y+q.z*q.w)*2, (q.z*q.x-q.y*q.w)*2};
}

template <class T>
inline vec<T, 3>
qydir (vec<T, 4> q) {
    return {(q.x*q.y-q.z*q.w)*2, q.w*q.w-q.x*q.x+q.y*q.y-q.z*q.z, (q.y*q.z+q.x*q.w)*2};
}

template <class T>
inline vec<T, 3>
qzdir (vec<T, 4> q) {
    return {(q.z*q.x+q.y*q.w)*2, (q.y*q.z-q.x*q.w)*2, q.w*q.w-q.x*q.x-q.y*q.y+q.z*q.z};
}

template <class T>
inline matrix<T, 3, 3>
qmat(vec<T, 4> q) {
    return matrix<T, 3, 3>(qxdir(q), qydir(q), qzdir(q));
}

// MARK: Diagonlization

template <class T>
inline vec<T, 4>
diagonalize(matrix<T, 3, 3> A)
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
    vec<T, 4> q(0, 0, 0, 1);
    for (i = 0; i<maxsteps; i++)
    {
        matrix<T, 3, 3> Q = qmat(q); // Q*v == q*v*conj(q)
        matrix<T, 3, 3> D = transpose(Q) * A * Q;  // A = Q*D*Q^T
        vec<T, 3> offdiag(D[1][2], D[0][2], D[0][1]); // elements not on the diagonal
        vec<T, 3> om(fabs(offdiag.x), fabs(offdiag.y), fabs(offdiag.z)); // mag of each offdiag elem
        int k = (om.x>om.y && om.x>om.z) ? 0 : (om.y>om.z) ? 1 : 2; // index of largest element of offdiag
        T thet;
        if (om.x>om.y && om.x>om.z) {
            thet = (D[2][2] - D[1][1]) / (2.0f*offdiag[0]);
        } else if (om.y>om.z) {
            thet = (D[0][0] - D[2][2]) / (2.0f*offdiag[1]);
        } else {
            thet = (D[1][1] - D[0][0]) / (2.0f*offdiag[2]);
        }
        if (offdiag[k] == 0.0f) break;  // diagonal already
        T sgn = sign(thet);
        thet *= sgn; // make it positive
        T t = sgn / (thet + ((thet<1.E6f) ? sqrt(thet*thet + 1.0f) : thet)); // sign(T)/(|T|+sqrt(T^2+1))
        T c = rsqrt(t*t + 1.0f); //  c= 1/(t^2+1) , t=s/c
        if (c == 1.0f) break;  // no room for improvement - reached machine precision.
        vec<T, 4> jr(0, 0, 0, 0); // jacobi rotation for this iteration.
        jr[k] = sgn*sqrt((1.0f - c) / 2.0f);  // using 1/2 angle identity sin(a/2) = sqrt((1-cos(a))/2)
        jr[k] *= -1.0f; // note we want a final result semantic that takes D to A, not A to D
        jr.w = sqrt(1.0f - (jr[k] * jr[k]));
        if (jr.w == 1.0f) break; // reached limits of floating point precision
        q = qmul(q, jr);
        q = normalize(q);
    }
    T h = 1.0f/sqrt(2.0f);  // M_SQRT1_2
    matrix<T, 3, 3> M = transpose(qmat(q)) * A * qmat(q);
    vec<T, 3> e = vec<T, 3>(M[0][0],M[1][1],M[2][2]);
    q = (e.x < e.z)  ? qmul(q, vec<T, 4>( 0, h, 0, h )) : q;
    q = (e.y < e.z)  ? qmul(q, vec<T, 4>( h, 0, 0, h )) : q;
    q = (e.x < e.y)  ? qmul(q, vec<T, 4>( 0, 0, h, h )) : q;   // size order z,y,x so xy spans a planeish spread
    q = (qzdir(q).z < 0) ? qmul(q, vec<T, 4>( 1, 0, 0, 0 )) : q;
    q = (qydir(q).y < 0) ? qmul(q, vec<T, 4>( 0, 0, 1, 0 )) : q;
    q = (q.w < 0) ? -q : q;
    return q;
}

// MARK: General

template <class T>
inline T
sqr(T A)
{
    return A * A;
}

template <class T>
inline matrix<T, 3, 3>
crossMatrix(vec<T, 3> v)
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
    T l00 = sqrt(a00);
    T l01 = a01/l00;
    T l02 = a02/l00;

    T l11 = sqrt(a11 - sqr(l01));
    T l12 = (a12 - l02 * l01) / l11;

    T l22 = sqrt(a22 - sqr(l02) - sqr(l12));

    // Store L into memory
    L[0][0] = l00;
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
    return (1.0 / determinant(m)) *
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
    switch (differentialSolution.type) {
        case QuadraticSolutionTypeComplex: {
            T c1 = differentialSolution.c1;
            T c2 = differentialSolution.c2;
            T λ = differentialSolution.x;
            T μ = differentialSolution.y;
            T k = differentialSolution.z;

            T y = c1*pow(e,λ*t)*cos(μ*t) + c2*pow(e,λ*t)*sin(μ*t) + k;
            T y_ddt = λ*c1*pow(e,λ*t)*cos(μ*t) - μ*c1*pow(e,λ*t)*sin(μ*t) +
            λ*c2*pow(e,λ*t)*sin(μ*t) + μ*c2*pow(e,λ*t)*cos(μ*t);
            T y_d2dt = λ*λ*c1*pow(e,λ*t)*cos(μ*t) - μ*λ*c1*pow(e,λ*t)*sin(μ*t) -
            (λ*μ*c1*pow(e,λ*t)*sin(μ*t) + μ*μ*c1*pow(e,λ*t)*cos(μ*t)) +
            λ*λ*c2*pow(e,λ*t)*sin(μ*t) + μ*λ*c2*pow(e,λ*t)*cos(μ*t) +
            λ*μ*c2*pow(e,λ*t)*cos(μ*t) - μ*μ*c2*pow(e,λ*t)*sin(μ*t);
            return vec<T, 3>(y, y_ddt, y_d2dt);
        }
        case QuadraticSolutionTypeReal: {
            T c1 = differentialSolution.c1;
            T c2 = differentialSolution.c2;
            T r = differentialSolution.x;
            T k = differentialSolution.y;

            T y = c1*pow(e,r*t) + c2*t*pow(e,r*t) + k;
            T y_ddt = r*c1*pow(e,r*t) +
            c2*pow(e,r*t) + r*c2*t*pow(e,r*t);
            T y_d2dt = r*r*c1*pow(e,r*t) +
            r*c2*pow(e,r*t) +
            r*c2*pow(e,r*t) + r*r*c2*t*pow(e,r*t);
            return vec<T, 3>(y, y_ddt, y_d2dt);
        }
        case QuadraticSolutionTypeRealDistinct: {
            T c1 = differentialSolution.c1;
            T c2 = differentialSolution.c2;
            T r1 = differentialSolution.x;
            T r2 = differentialSolution.y;
            T k = differentialSolution.z;

            T y = c1*pow(e,r1*t) + c2*pow(e,r2*t) + k;
            T y_ddt = r1*c1*pow(e,r1*t) + r2*c2*pow(e,r2*t);
            T y_d2dt = r1*r1*c1 * pow(e,r1*t) + r2*r2*c2 * pow(e,r2*t);
            return vec<T, 3>(y, y_ddt, y_d2dt);
        }
    }
}

template <class T>
inline vec<T, 3>
evaluateDifferential(T a, T b, T c, T g, T y_0, T y_ddt_0, T t)
{
    DifferentialSolution<T> differentialSolution = solveDifferential(a, b, c, g, y_0, y_ddt_0);
    return evaluateDifferential(differentialSolution, t);
}
