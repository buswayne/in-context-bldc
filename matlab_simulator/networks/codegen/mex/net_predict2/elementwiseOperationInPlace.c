/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * elementwiseOperationInPlace.c
 *
 * Code generation for function 'elementwiseOperationInPlace'
 *
 */

/* Include files */
#include "elementwiseOperationInPlace.h"
#include "net_predict2_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Function Definitions */
void lambdaForColumnMajorGeneric(real32_T X[10])
{
  int32_T iElem;
  for (iElem = 0; iElem < 10; iElem++) {
    X[iElem] = muSingleScalarExp(X[iElem]);
  }
}

/* End of code generation (elementwiseOperationInPlace.c) */
