#include <metal_stdlib>
using namespace metal;

// Quaternion and diagonalization ported from https://github.com/melax/sandbox/blob/3e267f2db2262a4cc6bf3f576c8c92b3cba79efc/include/geometric.h

template <class T>
static inline vec<T, 4>
qmul(vec<T, 4> a, vec<T, 4> b) {
    return vec<T, 4>(
                     a.x*b.w+a.w*b.x+a.y*b.z-a.z*b.y,
                     a.y*b.w+a.w*b.y+a.z*b.x-a.x*b.z,
                     a.z*b.w+a.w*b.z+a.x*b.y-a.y*b.x,
                     a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z);
}

template <class T>
static inline vec<T, 3>
qxdir(vec<T, 4> q) {
    return {q.w*q.w+q.x*q.x-q.y*q.y-q.z*q.z, (q.x*q.y+q.z*q.w)*2, (q.z*q.x-q.y*q.w)*2};
}

template <class T>
static inline vec<T, 3>
qydir (vec<T, 4> q) {
    return {(q.x*q.y-q.z*q.w)*2, q.w*q.w-q.x*q.x+q.y*q.y-q.z*q.z, (q.y*q.z+q.x*q.w)*2};
}

template <class T>
static inline vec<T, 3>
qzdir (vec<T, 4> q) {
    return {(q.z*q.x+q.y*q.w)*2, (q.y*q.z-q.x*q.w)*2, q.w*q.w-q.x*q.x-q.y*q.y+q.z*q.z};
}

template <class T>
static inline matrix<T, 3, 3>
qmat(vec<T, 4> q) {
    return matrix<T, 3, 3>(qxdir(q), qydir(q), qzdir(q));
}

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

struct EigenResult { float3x3 Q; float3 w; };

// Constants
#define M_SQRT3    1.73205080756887729352744634151   // sqrt(3)

// Macros
#define SQR(x)      ((x)*(x))                        // x^2

// ----------------------------------------------------------------------------
inline float3 dsyevc3(const float3x3 A)
// ----------------------------------------------------------------------------
// Calculates the eigenvalues of a symmetric 3x3 matrix A using Cardano's
// analytical algorithm.
// Only the diagonal and upper triangular parts of A are accessed. The access
// is read-only.
// ----------------------------------------------------------------------------
// Parameters:
//   A: The symmetric input matrix
// ----------------------------------------------------------------------------
// Return value:
//   Storage buffer for eigenvalues
// ----------------------------------------------------------------------------
{
    float3 w;
    float m, c1, c0;

    // Determine coefficients of characteristic poynomial. We write
    //       | a   d   f  |
    //  A =  | d*  b   e  |
    //       | f*  e*  c  |
    float de = A[0][1] * A[1][2];                                    // d * e
    float dd = SQR(A[0][1]);                                         // d^2
    float ee = SQR(A[1][2]);                                         // e^2
    float ff = SQR(A[0][2]);                                         // f^2
    m  = A[0][0] + A[1][1] + A[2][2];
    c1 = (A[0][0]*A[1][1] + A[0][0]*A[2][2] + A[1][1]*A[2][2])        // a*b + a*c + b*c - d^2 - e^2 - f^2
    - (dd + ee + ff);
    c0 = A[2][2]*dd + A[0][0]*ee + A[1][1]*ff - A[0][0]*A[1][1]*A[2][2]
    - 2.0 * A[0][2]*de;                                     // c*d^2 + a*e^2 + b*f^2 - a*b*c - 2*f*d*e)

    float p, sqrt_p, q, c, s, phi;
    p = SQR(m) - 3.0*c1;
    q = m*(p - (3.0/2.0)*c1) - (27.0/2.0)*c0;
    sqrt_p = sqrt(fabs(p));

    phi = 27.0 * ( 0.25*SQR(c1)*(p - c1) + c0*(q + 27.0/4.0*c0));
    phi = (1.0/3.0) * atan2(sqrt(fabs(phi)), q);

    c = sqrt_p*cos(phi);
    s = (1.0/M_SQRT3)*sqrt_p*sin(phi);

    w[1]  = (1.0/3.0)*(m - c);
    w[2]  = w[1] + s;
    w[0]  = w[1] + c;
    w[1] -= s;

    return w;
}

// ----------------------------------------------------------------------------
inline auto dsytrd3(float3x3 const A)
// ----------------------------------------------------------------------------
// Reduces a symmetric 3x3 matrix to tridiagonal form by applying
// (unitary) Householder transformations:
//            [ d[0]  e[0]       ]
//    A = Q . [ e[0]  d[1]  e[1] ] . Q^T
//            [       e[1]  d[2] ]
// The function accesses only the diagonal and upper triangular parts of
// A. The access is read-only.
// ---------------------------------------------------------------------------
{
    struct Result { float3x3 Q; float3 d; float3 e; };

    float3 u, q, d, e;
    float omega, f;
    float K, h, g;

    // Initialize Q to the identitity matrix
    float3x3 Q = float3x3(1);

    // Bring first row and column to the desired form
    h = SQR(A[0][1]) + SQR(A[0][2]);
    if (A[0][1] > 0)
        g = -sqrt(h);
    else
        g = sqrt(h);
    e[0] = g;
    f    = g * A[0][1];
    u[1] = A[0][1] - g;
    u[2] = A[0][2];

    omega = h - f;
    if (omega > 0.0) {
        omega = 1.0 / omega;
        K     = 0.0;

        f = A[1][1] * u[1] + A[1][2] * u[2];
        q[1] = omega * f;
        K += u[1] * f;

        f = A[1][2] * u[1] + A[2][2] * u[2];
        q[2] = omega * f;
        K += u[2] * f;

        K *= 0.5 * SQR(omega);

        q = q - K * q;
        d = A[0] - 2.0 * q * u;

        // Store inverse Householder transformation in Q
        Q[1] = float3(0,1,0) - omega * u[1]*u;
        Q[2] = float3(0,0,1) - omega * u[1]*u;

        // Calculate updated A[1][2] and store it in e[1]
        e[1] = A[1][2] - q[1]*u[2] - u[1]*q[2];
    } else {
        d = float3(A[0][0], A[1][1], A[2][2]);
        e[1] = A[1][2];
    }

    return Result { Q, d, e };
}

// ----------------------------------------------------------------------------
inline EigenResult dsyevq3(float3x3 const A)
// ----------------------------------------------------------------------------
// Calculates the eigenvalues and normalized eigenvectors of a symmetric 3x3
// matrix A using the QL algorithm with implicit shifts, preceded by a
// Householder reduction to tridiagonal form.
// The function accesses only the diagonal and upper triangular parts of A.
// The access is read-only.
// ----------------------------------------------------------------------------
// Parameters:
//   A: The symmetric input matrix
//   Q: Storage buffer for eigenvectors
//   w: Storage buffer for eigenvalues
// ----------------------------------------------------------------------------
// Dependencies:
//   dsytrd3()
// ----------------------------------------------------------------------------
{
    struct Result { float3x3 Q; float3 w; };

    const int n = 3;
    float g, r, p, f, b, s, c; // Intermediate storage
    int nIter;
    int m;

    // Transform A to real tridiagonal form by the Householder method
    auto tridiagonal = dsytrd3(A);
    float3x3 Q = tridiagonal.Q;
    float3 w = tridiagonal.d;
    float3 e = tridiagonal.e;

    // Calculate eigensystem of the remaining real symmetric tridiagonal matrix
    // with the QL method
    //
    // Loop over all off-diagonal elements
    for (int l=0; l < n-1; l++) {
        nIter = 0;
        while (1) {
            // Check for convergence and exit iteration loop if off-diagonal
            // element e(l) is zero
            for (m=l; m <= n-2; m++) {
                g = fabs(w[m])+fabs(w[m+1]);
                if (fabs(e[m]) + g == g)
                    break;
            }
            if (m == l)
                break;

            if (nIter++ >= 30)
                return EigenResult {};

            // Calculate g = d_m - k
            g = (w[l+1] - w[l]) / (e[l] + e[l]);
            r = sqrt(SQR(g) + 1.0);
            if (g > 0)
                g = w[m] - w[l] + e[l]/(g + r);
            else
                g = w[m] - w[l] + e[l]/(g - r);

            s = c = 1.0;
            p = 0.0;
            for (int i=m-1; i >= l; i--) {
                f = s * e[i];
                b = c * e[i];
                if (fabs(f) > fabs(g)) {
                    c      = g / f;
                    r      = sqrt(SQR(c) + 1.0);
                    e[i+1] = f * r;
                    c     *= (s = 1.0/r);
                } else {
                    s      = f / g;
                    r      = sqrt(SQR(s) + 1.0);
                    e[i+1] = g * r;
                    s     *= (c = 1.0/r);
                }

                g = w[i+1] - p;
                r = (w[i] - g)*s + 2.0*c*b;
                p = s * r;
                w[i+1] = g + p;
                g = c*r - b;

                float3 t = Q[i+1];
                float3 qi = Q[i];
                Q[i+1] = s*qi + c*t;
                Q[i]   = c*qi - s*t;
            }
            w[l] -= p;
            e[l]  = g;
            e[m]  = 0.0;
        }
    }

    return EigenResult { Q, w };
}

// ----------------------------------------------------------------------------
inline EigenResult dsyevh3(float3x3 const A)
// ----------------------------------------------------------------------------
// Calculates the eigenvalues and normalized eigenvectors of a symmetric 3x3
// matrix A using Cardano's method for the eigenvalues and an analytical
// method based on vector cross products for the eigenvectors. However,
// if conditions are such that a large error in the results is to be
// expected, the routine falls back to using the slower, but more
// accurate QL algorithm. Only the diagonal and upper triangular parts of A need
// to contain meaningful values. Access to A is read-only.
// ----------------------------------------------------------------------------
// Parameters:
//   A: The symmetric input matrix
//   Q: Storage buffer for eigenvectors
//   w: Storage buffer for eigenvalues
// ----------------------------------------------------------------------------
// Return value:
//   0: Success
//  -1: Error
// ----------------------------------------------------------------------------
// Dependencies:
//   dsyevc3(), dsytrd3(), dsyevq3()
// ----------------------------------------------------------------------------
// Version history:
//   v1.1: Simplified fallback condition --> speed-up
//   v1.0: First released version
// ----------------------------------------------------------------------------
{
    float3x3 Q;
    float norm;          // Squared norm or inverse norm of current eigenvector
    float error;         // Estimated maximum roundoff error
    float t, u;          // Intermediate storage

    // Calculate eigenvalues
    float3 w = dsyevc3(A);

    float3 abs_w = abs(w);
    t = fabs(w[0]);
    if ((u=abs_w[1]) > t)
        t = u;
    if ((u=abs_w[2]) > t)
        t = u;
    if (t < 1.0)
        u = t;
    else
        u = SQR(t);
    error = 256.0 * FLT_EPSILON * SQR(u);

    // Calculate first eigenvector by the formula
    //   v[0] = (A - w[0]).e1 x (A - w[0]).e2
    Q[0] = cross(A[0] - float3(w.x, 0, 0), A[1] - float3(0, w.x, 0));
    norm = dot(Q[0], Q[0]);

    // If vectors are nearly linearly dependent, or if there might have
    // been large cancellations in the calculation of A[i][i] - w[0], fall
    // back to QL algorithm
    // Note that this simultaneously ensures that multiple eigenvalues do
    // not cause problems: If w[0] = w[1], then A - w[0] * I has rank 1,
    // i.e. all columns of A - w[0] * I are linearly dependent.
    if (norm <= error)
        return dsyevq3(A);
    else {                     // This is the standard branch
        norm = sqrt(1.0 / norm);
        Q[0] *= norm;
    }

    // Calculate second eigenvector by the formula
    //   v[1] = (A - w[1]).e1 x (A - w[1]).e2
    Q[1] = cross(A[0] - float3(w.y, 0, 0), A[1] - float3(0, w.y, 0));
    norm     = dot(Q[1], Q[1]);
    if (norm <= error)
        return dsyevq3(A);
    else {
        norm = sqrt(1.0 / norm);
        Q[1] *= norm;
    }

    // Calculate third eigenvector according to
    //   v[2] = v[0] x v[1]
    Q[2] = cross(Q[0], Q[1]);

    return EigenResult { Q, w };
}

