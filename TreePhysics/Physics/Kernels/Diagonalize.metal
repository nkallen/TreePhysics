#include <metal_stdlib>
#include "ShaderTypes.h"
#include "Math.h"
#include "Math.metal"

using namespace metal;

inline quatf
diagonalize(float3x3 A) {
    const float Four_Gamma_Squared=sqrt(8.)+3.;
    const float Sine_Pi_Over_Eight=.5*sqrt(2.-sqrt(2.));
    const float Cosine_Pi_Over_Eight=.5*sqrt(2.+sqrt(2.));
    const float Tiny_Number = 1.e-20;

    float a00 = A.columns[0][0];
    float a01 = A.columns[0][1], a11 = A.columns[1][1];
    float a02 = A.columns[0][2], a12 = A.columns[1][2], a22 = A.columns[2][2];

    quatf q(0, 0, 0, 1);

    for (int sweep = 0; sweep < 4; sweep++) {
#define A00  a00
#define A01  a01
#define A02  a02
#define A11  a11
#define A12  a12
#define A22  a22
#define QIX  q.x
#define QIY  q.y
#define QIZ  q.z
#define TMP1 tmp.x
#define TMP2 tmp.y
#define TMP3 tmp.z
        {
#include "JacobianSweep.h"
        }
#undef A00
#undef A01
#undef A02
#undef A11
#undef A12
#undef A22
#undef QIX
#undef QIY
#undef QIZ
#undef TMP1
#undef TMP2
#undef TMP3

#define A00  a11
#define A01  a12
#define A02  a01
#define A11  a22
#define A12  a02
#define A22  a00
#define QIX  q.y
#define QIY  q.z
#define QIZ  q.x
#define TMP1 tmp.y
#define TMP2 tmp.z
#define TMP3 tmp.x
        {
#include "JacobianSweep.h"
        }
#undef A00
#undef A01
#undef A02
#undef A11
#undef A12
#undef A22
#undef QIX
#undef QIY
#undef QIZ
#undef TMP1
#undef TMP2
#undef TMP3

#define A00  a22
#define A01  a02
#define A02  a12
#define A11  a00
#define A12  a01
#define A22  a11
#define QIX  q.z
#define QIY  q.x
#define QIZ  q.y
#define TMP1 tmp.z
#define TMP2 tmp.x
#define TMP3 tmp.y
        {
#include "JacobianSweep.h"
        }
#undef A00
#undef A01
#undef A02
#undef A11
#undef A12
#undef A22
#undef QIX
#undef QIY
#undef QIZ
#undef TMP1
#undef TMP2
#undef TMP3
    }

    return normalize(q);
}

