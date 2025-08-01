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
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include "omp.h"

/* Function Definitions */
/*
 *
 */
void lambdaForColumnMajorGeneric(const emlrtStack *sp, real32_T X[10])
{
  jmp_buf *volatile emlrtJBStack;
  int32_T iElem;
  int32_T lambdaForColumnMajorGeneric_numThreads;
  emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
  emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
  lambdaForColumnMajorGeneric_numThreads = emlrtAllocRegionTLSs(
      sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(lambdaForColumnMajorGeneric_numThreads)

  for (iElem = 0; iElem < 10; iElem++) {
    X[iElem] = muSingleScalarExp(X[iElem]);
  }
  emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
  emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
}

/* End of code generation (elementwiseOperationInPlace.c) */
