import Foundation
import simd

fileprivate let sqrt_3: Float = 1.73205080756887729352744634151

extension matrix_float3x3 {
    // Calculates the eigenvalues of a symmetric 3x3 matrix A using Cardano's
    // analytical algorithm.
    var eigenvalues_cardano: float3 {
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
}
