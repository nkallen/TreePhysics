//
//  Eigensolver.h
//  TreePhysics
//
//  Created by Nick Kallen on 10/31/19.
//  Copyright © 2019 Nick Kallen. All rights reserved.
//

#ifndef Eigensolver_h
#define Eigensolver_h

#import <simd/simd.h>

typedef struct {
    simd_float3 eigenvalues;
    simd_float3x3 eigenvectors;
} Eigen;

Eigen eigenvalues(simd_float3 ltr, simd_float3 diag);


#endif /* Eigensolver_h */
