/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_net_predict_mex.c
 *
 * Code generation for function '_coder_net_predict_mex'
 *
 */

/* Include files */
#include "_coder_net_predict_mex.h"
#include "_coder_net_predict_api.h"
#include "net_predict.h"
#include "net_predict_data.h"
#include "net_predict_initialize.h"
#include "net_predict_terminate.h"
#include "rt_nonfinite.h"
#include "omp.h"

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  static jmp_buf emlrtJBEnviron;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexAtExit(&net_predict_atexit);
  /* Initialize the memory manager. */
  omp_init_lock(&emlrtLockGlobal);
  omp_init_nest_lock(&net_predict_nestLockGlobal);
  net_predict_initialize();
  st.tls = emlrtRootTLSGlobal;
  emlrtSetJmpBuf(&st, &emlrtJBEnviron);
  if (setjmp(emlrtJBEnviron) == 0) {
    net_predict_mexFunction(nlhs, plhs, nrhs, prhs);
    net_predict_terminate();
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&net_predict_nestLockGlobal);
  } else {
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&net_predict_nestLockGlobal);
    emlrtReportParallelRunTimeError(&st);
  }
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal,
                           &emlrtLockerFunction, omp_get_num_procs(), NULL,
                           "windows-1252", true);
  return emlrtRootTLSGlobal;
}

void net_predict_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
                             const mxArray *prhs[1])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 1, 4,
                        11, "net_predict");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 11,
                        "net_predict");
  }
  /* Call the function. */
  net_predict_api(prhs[0], &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

/* End of code generation (_coder_net_predict_mex.c) */
