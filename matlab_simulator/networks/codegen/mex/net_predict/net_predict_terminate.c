/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * net_predict_terminate.c
 *
 * Code generation for function 'net_predict_terminate'
 *
 */

/* Include files */
#include "net_predict_terminate.h"
#include "_coder_net_predict_mex.h"
#include "net_predict_data.h"
#include "rt_nonfinite.h"
#include "omp.h"

/* Function Declarations */
static void emlrtExitTimeCleanupDtorFcn(const void *r);

/* Function Definitions */
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void net_predict_atexit(void)
{
  static jmp_buf emlrtJBEnviron;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  /* Initialize the memory manager. */
  omp_init_lock(&emlrtLockGlobal);
  omp_init_nest_lock(&net_predict_nestLockGlobal);
  st.tls = emlrtRootTLSGlobal;
  emlrtSetJmpBuf(&st, &emlrtJBEnviron);
  if (setjmp(emlrtJBEnviron) == 0) {
    emlrtPushHeapReferenceStackR2021a(&st, false, NULL,
                                      (void *)&emlrtExitTimeCleanupDtorFcn,
                                      NULL, NULL, NULL);
    emlrtEnterRtStackR2012b(&st);
    emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
    emlrtExitTimeCleanup(&emlrtContextGlobal);
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&net_predict_nestLockGlobal);
  } else {
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&net_predict_nestLockGlobal);
    emlrtReportParallelRunTimeError(&st);
  }
}

void net_predict_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (net_predict_terminate.c) */
