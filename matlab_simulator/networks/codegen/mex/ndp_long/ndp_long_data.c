/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ndp_long_data.c
 *
 * Code generation for function 'ndp_long_data'
 *
 */

/* Include files */
#include "ndp_long_data.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                /* bFirstTime */
    false,                                               /* bInitialized */
    131659U,                                             /* fVersionInfo */
    NULL,                                                /* fErrorFunction */
    "ndp_long",                                          /* fFunctionName */
    NULL,                                                /* fRTCallStack */
    false,                                               /* bDebugMode */
    {925302475U, 3951113832U, 2887047743U, 3194870166U}, /* fSigWrd */
    NULL                                                 /* fSigMem */
};

omp_lock_t emlrtLockGlobal;

omp_nest_lock_t ndp_long_nestLockGlobal;

covrtInstance emlrtCoverageInstance;

/* End of code generation (ndp_long_data.c) */
