/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeSelfAttention.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Jan-2026 10:49:46
 */

#ifndef COMPUTESELFATTENTION_H
#define COMPUTESELFATTENTION_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void iInputLinearProjection(const float X[160], const float QW[256],
                            const float KW[256], const float VW[256],
                            const float QB[16], const float KB[16],
                            const float VB[16], float Q[160], float K[160],
                            float V[160]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for computeSelfAttention.h
 *
 * [EOF]
 */
