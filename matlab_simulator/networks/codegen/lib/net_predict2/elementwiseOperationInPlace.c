/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: elementwiseOperationInPlace.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 01-Sep-2025 14:59:08
 */

/* Include Files */
#include "elementwiseOperationInPlace.h"
#include "rt_nonfinite.h"
#include "omp.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : float X[10]
 * Return Type  : void
 */
void lambdaForColumnMajorGeneric(float X[10])
{
  int iElem;
#pragma omp parallel for num_threads(omp_get_max_threads())

  for (iElem = 0; iElem < 10; iElem++) {
    X[iElem] = expf(X[iElem]);
  }
}

/*
 * File trailer for elementwiseOperationInPlace.c
 *
 * [EOF]
 */
