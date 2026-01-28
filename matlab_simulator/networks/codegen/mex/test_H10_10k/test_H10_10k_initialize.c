/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * test_H10_10k_initialize.c
 *
 * Code generation for function 'test_H10_10k_initialize'
 *
 */

/* Include files */
#include "test_H10_10k_initialize.h"
#include "_coder_test_H10_10k_mex.h"
#include "rt_nonfinite.h"
#include "test_H10_10k.h"
#include "test_H10_10k_data.h"

/* Function Declarations */
static void test_H10_10k_once(void);

/* Function Definitions */
static void test_H10_10k_once(void)
{
  mex_InitInfAndNan();
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "C:\\Users\\39340\\Documents\\GitHub\\in-context-"
                  "bldc\\matlab_simulator\\networks\\test_H10_10k.m",
                  0U, 1U, 2U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 0U, 0U, "test_H10_10k", 0, -1, 581);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 1U, 503, -1, 579);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 0U, 411, -1, 478);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 0U, 0U, 389, 406, -1, 482);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 0U);
  test_H10_10k_init();
}

void test_H10_10k_initialize(void)
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
    test_H10_10k_once();
  }
}

/* End of code generation (test_H10_10k_initialize.c) */
