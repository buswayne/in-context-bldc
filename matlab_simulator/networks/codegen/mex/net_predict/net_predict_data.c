/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * net_predict_data.c
 *
 * Code generation for function 'net_predict_data'
 *
 */

/* Include files */
#include "net_predict_data.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                /* bFirstTime */
    false,                                               /* bInitialized */
    131659U,                                             /* fVersionInfo */
    NULL,                                                /* fErrorFunction */
    "net_predict",                                       /* fFunctionName */
    NULL,                                                /* fRTCallStack */
    false,                                               /* bDebugMode */
    {925302475U, 3951113832U, 2887047743U, 3194870166U}, /* fSigWrd */
    NULL                                                 /* fSigMem */
};

omp_lock_t emlrtLockGlobal;

omp_nest_lock_t net_predict_nestLockGlobal;

/* End of code generation (net_predict_data.c) */
