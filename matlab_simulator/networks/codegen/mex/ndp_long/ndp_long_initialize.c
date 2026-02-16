/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ndp_long_initialize.c
 *
 * Code generation for function 'ndp_long_initialize'
 *
 */

/* Include files */
#include "ndp_long_initialize.h"
#include "_coder_ndp_long_mex.h"
#include "ndp_long.h"
#include "ndp_long_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void ndp_long_once(void);

/* Function Definitions */
static void ndp_long_once(void)
{
  mex_InitInfAndNan();
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "C:\\Users\\39340\\Documents\\GitHub\\in-context-"
                  "bldc\\matlab_simulator\\networks\\ndp_long.m",
                  0U, 1U, 2U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 0U, 0U, "ndp_long", 0, -1, 569);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 1U, 491, -1, 567);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 0U, 407, -1, 466);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 0U, 0U, 385, 402, -1, 470);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 0U);
  ndp_long_init();
}

void ndp_long_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2022b(&st);
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtLicenseCheckR2022a(&st, "EMLRT:runTime:MexFunctionNeedsLicense",
                          "neural_network_toolbox", 2);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    ndp_long_once();
  }
}

/* End of code generation (ndp_long_initialize.c) */
