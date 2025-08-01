/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * net_predict_initialize.c
 *
 * Code generation for function 'net_predict_initialize'
 *
 */

/* Include files */
#include "net_predict_initialize.h"
#include "_coder_net_predict_mex.h"
#include "net_predict_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void net_predict_once(void);

/* Function Definitions */
static void net_predict_once(void)
{
  mex_InitInfAndNan();
}

void net_predict_initialize(void)
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
    net_predict_once();
  }
}

/* End of code generation (net_predict_initialize.c) */
