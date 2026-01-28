/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sumMatrixIncludeNaN.c
 *
 * Code generation for function 'sumMatrixIncludeNaN'
 *
 */

/* Include files */
#include "sumMatrixIncludeNaN.h"
#include "rt_nonfinite.h"
#include "test_H10_10k_data.h"

/* Function Definitions */
real32_T b_sumColumnB(const real32_T x[10])
{
  int32_T k;
  real32_T y;
  y = x[0];
  for (k = 0; k < 9; k++) {
    y += x[k + 1];
  }
  return y;
}

real32_T sumColumnB(const real32_T x[160], int32_T col)
{
  int32_T i0;
  int32_T k;
  real32_T y;
  i0 = (col - 1) << 4;
  y = x[i0];
  for (k = 0; k < 15; k++) {
    y += x[(i0 + k) + 1];
  }
  return y;
}

/* End of code generation (sumMatrixIncludeNaN.c) */
