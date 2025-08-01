/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_net_predict_api.c
 *
 * Code generation for function '_coder_net_predict_api'
 *
 */

/* Include files */
#include "_coder_net_predict_api.h"
#include "net_predict.h"
#include "net_predict_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[80];

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[80];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier))[80];

static const mxArray *emlrt_marshallOut(real32_T u[10]);

/* Function Definitions */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[80]
{
  real_T(*y)[80];
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[80]
{
  static const int32_T dims[3] = {1, 10, 8};
  real_T(*ret)[80];
  int32_T iv[3];
  boolean_T bv[3] = {false, false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 3U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret = (real_T(*)[80])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier))[80]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[80];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static const mxArray *emlrt_marshallOut(real32_T u[10])
{
  static const int32_T iv[3] = {0, 0, 0};
  static const int32_T iv1[3] = {1, 1, 10};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(3, (const void *)&iv[0], mxSINGLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 3);
  emlrtAssign(&y, m);
  return y;
}

void net_predict_api(const mxArray *prhs, const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*in)[80];
  real32_T(*out)[10];
  st.tls = emlrtRootTLSGlobal;
  out = (real32_T(*)[10])mxMalloc(sizeof(real32_T[10]));
  /* Marshall function inputs */
  in = emlrt_marshallIn(&st, emlrtAlias(prhs), "in");
  /* Invoke the target function */
  net_predict(&st, *in, *out);
  /* Marshall function outputs */
  *plhs = emlrt_marshallOut(*out);
}

/* End of code generation (_coder_net_predict_api.c) */
