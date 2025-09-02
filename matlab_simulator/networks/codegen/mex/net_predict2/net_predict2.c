/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * net_predict2.c
 *
 * Code generation for function 'net_predict2'
 *
 */

/* Include files */
#include "net_predict2.h"
#include "net_predict2_data.h"
#include "predict.h"
#include "rt_nonfinite.h"
#include "omp.h"

/* Variable Definitions */
static boolean_T mynet_not_empty;

static emlrtRSInfo emlrtRSI = {
    17,             /* lineNo */
    "net_predict2", /* fcnName */
    "C:\\Users\\39340\\Documents\\GitHub\\in-context-bldc\\matlab_"
    "simulator\\networks\\net_predict2.m" /* pathName */
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

void net_predict2(const emlrtStack *sp, const real_T in[80], real32_T out[10])
{
  emlrtStack st;
  int32_T i;
  real32_T dl_in_Data[80];
  st.prev = sp;
  st.tls = sp->tls;
  covrtLogFcn(&emlrtCoverageInstance, 0U, 0U);
  /*  A persistent object mynet is used to load the series network object. */
  /*  At the first call to this function, the persistent object is constructed
   * and */
  /*  setup. When the function is called subsequent times, the same object is
   * reused  */
  /*  to call predict on inputs, thus avoiding reconstructing and reloading the
   */
  /*  network object. */
  if (covrtLogIf(&emlrtCoverageInstance, 0U, 0U, 0, !mynet_not_empty)) {
    covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 0U);
    mynet_not_empty = true;
  }
  covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 1U);
  /*  pass in input    */
  for (i = 0; i < 80; i++) {
    dl_in_Data[i] = (real32_T)in[i];
  }
  st.site = &emlrtRSI;
  dlnetwork_predict(&st, dl_in_Data, out);
}

void net_predict2_init(void)
{
  mynet_not_empty = false;
}

/* End of code generation (net_predict2.c) */
