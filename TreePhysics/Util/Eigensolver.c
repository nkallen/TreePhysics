#import <simd/simd.h>
#import "Eigensolver.h"

static simd_float3 computeEigenvector0(simd_float3 ltr, simd_float3 diag, float eigenvalue0) {
    float a01 = ltr.x, a02 = ltr.y, a12 = ltr.z;
    float a00 = diag.x, a11 = diag.y, a22 = diag.z;

    simd_float3 row0 = { a00 - eigenvalue0, a01, a02 };
    simd_float3 row1 = { a01, a11 - eigenvalue0, a12 };
    simd_float3 row2 = { a02, a12, a22 - eigenvalue0 };
    simd_float3 r0xr1 = simd_cross(row0, row1);
    simd_float3 r0xr2 = simd_cross(row0, row2);
    simd_float3 r1xr2 = simd_cross(row1, row2);
    float d0 = simd_dot(r0xr1, r0xr1);
    float d1 = simd_dot(r0xr2, r0xr2);
    float d2 = simd_dot(r1xr2, r1xr2);

    int d0_or_d1_mask = 0 - (d1 > d0);
    int d1_or_d2_mask = 0 - (d2 > d1);
    simd_float3 result = simd_select(r0xr1 * simd_precise_rsqrt(d0), r0xr2 * simd_precise_rsqrt(d1), d0_or_d1_mask);
    result = simd_select(result, r1xr2 * simd_precise_rsqrt(d2), d1_or_d2_mask);
    return result;
}

static simd_float2x3 computeOrthogonalComplement(simd_float3 w) {
    float invLength;
    simd_float3 u;
    if (fabs(w.x > fabs(w.y))) {
        invLength = simd_precise_rsqrt(simd_length_squared((simd_float2){ w.x, w.z }));
        u = (simd_float3){ -w.z * invLength, 0, +w.x * invLength };
    } else {
        invLength = simd_precise_rsqrt(simd_length_squared((simd_float2){ w.y, w.z }));
        u = (simd_float3){ 0, +w.z * invLength, -w.y * invLength };
    }
    simd_float3 v = simd_cross(w, u);
    return (simd_float2x3){ u, v };
}

static simd_float3 computeEigenvector1(simd_float3 ltr, simd_float3 diag, simd_float3 eigenvector0, float eigenvalue1) {
    float a01 = ltr.x, a02 = ltr.y, a12 = ltr.z;
    float a00 = diag.x, a11 = diag.y, a22 = diag.z;

    simd_float2x3 uv = computeOrthogonalComplement(eigenvector0);
    simd_float3 u = uv.columns[0];
    simd_float3 v = uv.columns[1];
    simd_float3x3 A = {
        .columns = {
            { a00, a01, a02 },
            { a01, a11, a12 },
            { a02, a12, a22 },
        }
    };

    simd_float3 Au = matrix_multiply(A, u);
    simd_float3 Av = matrix_multiply(A, v);
    float m00 = simd_dot(u, Au) - eigenvalue1;
    float m01 = simd_dot(u, Av);
    float m11 = simd_dot(v, Av) - eigenvalue1;
    float absM00 = fabs(m00);
    float absM01 = fabs(m01);
    float absM11 = fabs(m11);
    float maxAbsComp;
    if (absM00 >= absM11) {
        maxAbsComp = simd_max(absM00, absM01);
        if (maxAbsComp > 0) {
            if (absM00 >= absM01) {
                m01 /= m00;
                m00 = simd_precise_rsqrt(1.0 + m01*m01);
                m01 *= m00;
            } else {
                m00 /= m01;
                m01 = simd_precise_rsqrt(1 + m00*m00);
                m00 *= m01;
            }
            return m01 * u - m00 * v;
        } else {
            return u;
        }
    } else {
        maxAbsComp = simd_max(absM11, absM01);
        if (maxAbsComp > 0) {
            if (absM11 >= absM01) {
                m01 /= m11;
                m11 = simd_precise_rsqrt(1 + m01*m01);
                m01 *= m11;
            } else {
                m11 /= m01;
                m01 = simd_precise_rsqrt(1 + m11*m11);
                m11 *= m01;
            }
            return m11 * u - m01 * v;
        } else {
            return u;
        }
    }
}

Eigen eigenvalues(simd_float3 ltr, simd_float3 diag) {
    float maxAbsElement = simd_reduce_max(simd_max(simd_abs(ltr), simd_abs(diag)));
    if (maxAbsElement == 0.0) {
        return (Eigen) {
            .eigenvalues = {0, 0, 0},
            .eigenvectors = matrix_identity_float3x3
        };
    }

    float invMaxAbsElement = 1.0/maxAbsElement;
    ltr *= invMaxAbsElement;
    diag *= invMaxAbsElement;

    float norm = simd_dot(ltr, ltr);
    if (norm > 0) {
        float q = simd_reduce_add(diag) / 3.0;
        simd_float3 b = diag - q;
        float p = sqrt((simd_dot(b, b) + norm * 2.0) / 6.0);
        float c00 = b.y * b.z - ltr.z * ltr.z;
        float c01 = ltr.x * b.z - ltr.z * ltr.y;
        float c02 = ltr.x * ltr.z - b.y * ltr.y;
        float det = (b.x * c00 - ltr.x * c01 + ltr.y * c02) / (p*p*p);
        float halfDet = 0.5 * det;
        halfDet = simd_clamp(halfDet, -1.0f, 1.0f);
        float angle = acos(halfDet) / 3.0;
        float const twoThirdsPi = 2.09439510239319549f;
        float beta2 = cos(angle) * 2.0;
        float beta0 = cos(angle + twoThirdsPi) * 2.0;
        float beta1 = -(beta0 + beta2);
        simd_float3 beta = { beta0, beta1, beta2 };
        simd_float3 eigenvalues = q + p * beta;
        simd_float3 eigenvector0, eigenvector1, eigenvector2;
        if (halfDet >= 0) {
            eigenvector2 = computeEigenvector0(ltr, diag, eigenvalues.z);
            eigenvector1 = computeEigenvector1(ltr, diag, eigenvector2, eigenvalues.y);
            eigenvector0 = simd_cross(eigenvector1, eigenvector2);
        } else {
            eigenvector0 = computeEigenvector0(ltr, diag, eigenvalues.x);
            eigenvector1 = computeEigenvector1(ltr, diag, eigenvector0, eigenvalues.y);
            eigenvector2 = simd_cross(eigenvector0, eigenvector1);
        }
        return (Eigen){
            .eigenvalues = maxAbsElement * eigenvalues,
            .eigenvectors = { eigenvector0, eigenvector1, eigenvector2 }
        };
    } else {
        return (Eigen){
            .eigenvalues = diag * maxAbsElement,
            .eigenvectors = matrix_identity_float3x3
        };
    }
}
