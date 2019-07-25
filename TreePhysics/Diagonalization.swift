import Foundation
import simd

fileprivate let sqrt_3: Float = 1.73205080756887729352744634151
fileprivate let sqrt_3_d: Double = 1.73205080756887729352744634151

extension matrix_double3x3 {
    // Calculates the eigenvalues of a symmetric 3x3 matrix A using Cardano's
    // analytical algorithm.
    var eigenvalues_analytical: double3 {
        // Determine coefficients of characteristic poynomial. We write
        //       | a   d   f  |
        //  A =  | d*  b   e  |
        //       | f*  e*  c  |
        let de = self[1, 0] * self[2, 1]                                 // d * e
        let dd = sqr(self[1, 0])                                         // d^2
        let ee = sqr(self[2, 1])                                         // e^2
        let ff = sqr(self[2, 0])                                         // f^2
        let m  = self[0, 0] + self[1, 1] + self[2, 2]
        // a*b + a*c + b*c - d^2 - e^2 - f^2
        let c1 = (self[0, 0]*self[1, 1] + self[0, 0]*self[2, 2] + self[1, 1]*self[2, 2]) -
            (dd + ee + ff)
        // c*d^2 + a*e^2 + b*f^2 - a*b*c - 2*f*d*e)
        let c0 = self[2, 2]*dd + self[0, 0]*ee + self[1, 1]*ff - self[0, 0]*self[1, 1]*self[2, 2]
            - 2.0 * self[2, 0]*de

        let p = sqr(m) - 3.0*c1
        let q = m*(p - (3.0/2.0)*c1) - (27.0/2.0)*c0
        let sqrt_p = sqrt(abs(p))

        var phi = 27.0 * ( 0.25*sqr(c1)*(p - c1) + c0*(q + 27.0/4.0*c0))
        phi = (1.0/3.0) * atan2(sqrt(abs(phi)), q)

        let c = sqrt_p*cos(phi)
        let s = (1.0/sqrt_3_d)*sqrt_p*sin(phi)

        var w1  = (1.0/3.0)*(m - c)
        let w2  = w1 + s
        let w0  = w1 + c
        w1 -= s

        return double3(w0, w1, w2)
    }
}

extension matrix_float3x3 {
    // Calculates the eigenvalues of a symmetric 3x3 matrix A using Cardano's
    // analytical algorithm.
    var eigenvalues_analytical: float3 {
        // Determine coefficients of characteristic poynomial. We write
        //       | a   d   f  |
        //  A =  | d*  b   e  |
        //       | f*  e*  c  |
        let de = self[1, 0] * self[2, 1]                                 // d * e
        let dd = sqr(self[1, 0])                                         // d^2
        let ee = sqr(self[2, 1])                                         // e^2
        let ff = sqr(self[2, 0])                                         // f^2
        let m  = self[0, 0] + self[1, 1] + self[2, 2]
        // a*b + a*c + b*c - d^2 - e^2 - f^2
        let c1 = (self[0, 0]*self[1, 1] + self[0, 0]*self[2, 2] + self[1, 1]*self[2, 2]) -
            (dd + ee + ff)
        // c*d^2 + a*e^2 + b*f^2 - a*b*c - 2*f*d*e)
        let c0 = self[2, 2]*dd + self[0, 0]*ee + self[1, 1]*ff - self[0, 0]*self[1, 1]*self[2, 2]
            - 2.0 * self[2, 0]*de

        let p = sqr(m) - 3.0*c1
        let q = m*(p - (3.0/2.0)*c1) - (27.0/2.0)*c0
        let sqrt_p = sqrt(abs(p))

        var phi = 27.0 * ( 0.25*sqr(c1)*(p - c1) + c0*(q + 27.0/4.0*c0))
        phi = (1.0/3.0) * atan2(sqrt(abs(phi)), q)

        let c = sqrt_p*cos(phi)
        let s = (1.0/sqrt_3)*sqrt_p*sin(phi)

        var w1  = (1.0/3.0)*(m - c)
        let w2  = w1 + s
        let w0  = w1 + c
        w1 -= s

        return float3(w0, w1, w2)
    }

    var eigenvectors_analytical: float3x3 {
        var result = float3x3.init(0)
        let w = self.eigenvalues_analytical
        let abs_w = abs(w)

        var t = abs_w.x
        var u = abs_w.y
        if u > abs_w.x {
            t = abs_w.x
        }
        u = abs_w.z
        if u > t {
            t = u
        }
        if t < 1.0 {
            u = t
        } else {
            u = sqr(t)
        }
        let error = 256.0 * Float.ulpOfOne * sqr(u)

        result[1] = float3(
            self[1, 0]*self[2, 1] - self[2, 0]*self[1, 1],
            self[2, 0]*self[1, 0] - self[2, 1]*self[0, 0],
            sqr(self[1, 0]))

        // Calculate first eigenvector by the formula
        //   v[0] = (A - w[0]).e1 x (A - w[0]).e2
        result[0] = cross(self[0] - float3(w.x, 0, 0), self[1] - float3(0, w.x, 0))
        var norm = dot(result[0], result[0])

        // If vectors are nearly linearly dependent, or if there might have
        // been large cancellations in the calculation of A[i][i] - w[0], fall
        // back to QL algorithm
        // Note that this simultaneously ensures that multiple eigenvalues do
        // not cause problems: If w[0] = w[1], then A - w[0] * I has rank 1,
        // i.e. all columns of A - w[0] * I are linearly dependent.
        if norm <= error {
            fatalError("return dsyevq3(A, Q, w)")
        } else {                      // This is the standard branch
            norm = sqrt(1.0 / norm)
            result[0] *= norm
        }

        // Calculate second eigenvector by the formula
        //   v[1] = (A - w[1]).e1 x (A - w[1]).e2
        result[1] = cross(self[0] - float3(w.y, 0, 0), self[1] - float3(0, w.y, 0))

        norm = dot(result[1], result[1])
        if norm <= error {
            fatalError("return dsyevq3(A, Q, w)")
        } else {
            norm = sqrt(1.0 / norm)
            result[1] *= norm
        }

        // Calculate third eigenvector according to
        //   v[2] = v[0] x v[1]
        result[2] = cross(result[0], result[1])

        return result
    }
}
