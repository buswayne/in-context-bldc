/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * net_predict.c
 *
 * Code generation for function 'net_predict'
 *
 */

/* Include files */
#include "net_predict.h"
#include "callPredict.h"
#include "net_predict_data.h"
#include "rt_nonfinite.h"
#include "omp.h"

/* Variable Definitions */
static emlrtRSInfo emlrtRSI =
    {
        17,            /* lineNo */
        "net_predict", /* fcnName */
        "C:\\Users\\39340\\Documents\\GitHub\\ST-"
        "microelectronics\\matlab\\apps\\app_usb_sensored\\networks\\net_"
        "predict.m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI = {
    1,                   /* lineNo */
    "dlnetwork/predict", /* fcnName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\+ctarget\\@dlnetwork\\predict.p" /* pathName */
};

/* Function Definitions */
emlrtCTX emlrtGetRootTLSGlobal(void)
{
  return emlrtRootTLSGlobal;
}

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData)
{
  omp_set_lock(&emlrtLockGlobal);
  emlrtCallLockeeFunction(aLockee, aTLS, aData);
  omp_unset_lock(&emlrtLockGlobal);
}

/*
 * function out = net_predict(in)
 */
void net_predict(const emlrtStack *sp, const real_T in[80], real32_T out[10])
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T b_k;
  int32_T k;
  real32_T dataInputsSingle_0_f1[80];
  real32_T dl_in_Data[80];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  /*  A persistent object mynet is used to load the series network object. */
  /*  At the first call to this function, the persistent object is constructed
   * and */
  /*  setup. When the function is called subsequent times, the same object is
   * reused  */
  /*  to call predict on inputs, thus avoiding reconstructing and reloading the
   */
  /*  network object. */
  /* 'net_predict:11' if isempty(mynet) */
  /*  pass in input    */
  /* 'net_predict:16' dl_in = single(dlarray(in, 'BTC')); */
  for (k = 0; k < 80; k++) {
    dl_in_Data[k] = (real32_T)in[k];
  }
  /* 'net_predict:17' out = extractdata(predict(mynet,dl_in)); */
  st.site = &emlrtRSI;
  for (k = 0; k < 8; k++) {
    for (b_k = 0; b_k < 10; b_k++) {
      dataInputsSingle_0_f1[k + (b_k << 3)] = dl_in_Data[b_k + 10 * k];
    }
  }
  b_st.site = &g_emlrtRSI;
  predict(&b_st, dataInputsSingle_0_f1, out);
}

/* End of code generation (net_predict.c) */
