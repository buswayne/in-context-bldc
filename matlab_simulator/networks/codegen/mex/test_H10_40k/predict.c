/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * predict.c
 *
 * Code generation for function 'predict'
 *
 */

/* Include files */
#include "predict.h"
#include "callPredict.h"
#include "rt_nonfinite.h"
#include "test_H10_40k_data.h"

/* Variable Definitions */
static emlrtRSInfo g_emlrtRSI = {
    1,                   /* lineNo */
    "dlnetwork/predict", /* fcnName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\+ctarget\\@dlnetwork\\predict.p" /* pathName */
};

/* Function Definitions */
void dlnetwork_predict(const emlrtStack *sp, const real32_T varargin_1_Data[80],
                       real32_T varargout_1_Data[10])
{
  emlrtStack st;
  int32_T b_k;
  int32_T k;
  real32_T dataInputsSingle_0_f1[80];
  st.prev = sp;
  st.tls = sp->tls;
  for (k = 0; k < 8; k++) {
    for (b_k = 0; b_k < 10; b_k++) {
      dataInputsSingle_0_f1[k + (b_k << 3)] = varargin_1_Data[b_k + 10 * k];
    }
  }
  st.site = &g_emlrtRSI;
  predict(&st, dataInputsSingle_0_f1, varargout_1_Data);
}

/* End of code generation (predict.c) */
