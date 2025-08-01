/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * addBiasApplyActivation.c
 *
 * Code generation for function 'addBiasApplyActivation'
 *
 */

/* Include files */
#include "addBiasApplyActivation.h"
#include "rt_nonfinite.h"
#include "omp.h"

/* Variable Definitions */
static emlrtBCInfo c_emlrtBCI = {
    1,                        /* iFirst */
    16,                       /* iLast */
    1,                        /* lineNo */
    1,                        /* colNo */
    "",                       /* aName */
    "addBiasApplyActivation", /* fName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\+layer\\addBiasApplyActivation.p", /* pName */
    3                                           /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    1,                        /* iFirst */
    64,                       /* iLast */
    1,                        /* lineNo */
    1,                        /* colNo */
    "",                       /* aName */
    "addBiasApplyActivation", /* fName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\+layer\\addBiasApplyActivation.p", /* pName */
    3                                           /* checkKind */
};

/* Function Definitions */
/*
 *
 */
void addBiasApplyActivation(const emlrtStack *sp, real32_T X[160],
                            const real32_T bias[16])
{
  jmp_buf emlrtJBEnviron;
  jmp_buf *volatile emlrtJBStack;
  emlrtStack st;
  int32_T addBiasApplyActivation_numThreads;
  int32_T iElem;
  int32_T varargout_3;
  int32_T vk;
  boolean_T emlrtHadParallelError = false;
  emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
  emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
  addBiasApplyActivation_numThreads = emlrtAllocRegionTLSs(
      sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel num_threads(addBiasApplyActivation_numThreads) private(   \
        st, emlrtJBEnviron, vk, varargout_3)                                   \
    firstprivate(emlrtHadParallelError)
  {
    if (setjmp(emlrtJBEnviron) == 0) {
      st.prev = sp;
      st.tls = emlrtAllocTLS((emlrtCTX)sp, omp_get_thread_num());
      st.site = NULL;
      emlrtSetJmpBuf(&st, &emlrtJBEnviron);
    } else {
      emlrtHadParallelError = true;
    }
#pragma omp for nowait
    for (iElem = 0; iElem < 160; iElem++) {
      if (emlrtHadParallelError) {
        continue;
      }
      if (setjmp(emlrtJBEnviron) == 0) {
        vk = iElem >> 4;
        varargout_3 = (iElem - (vk << 4)) + 1;
        if ((varargout_3 < 1) || (varargout_3 > 16)) {
          emlrtDynamicBoundsCheckR2012b(varargout_3, 1, 16, &c_emlrtBCI, &st);
        }
        vk = (varargout_3 + (vk << 4)) - 1;
        X[vk] += bias[varargout_3 - 1];
      } else {
        emlrtHadParallelError = true;
      }
    }
  }
  emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
  emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
}

/*
 *
 */
void b_addBiasApplyActivation(const emlrtStack *sp)
{
  jmp_buf emlrtJBEnviron;
  jmp_buf *volatile emlrtJBStack;
  emlrtStack st;
  int32_T b_addBiasApplyActivation_numThreads;
  int32_T iElem;
  int32_T v1;
  boolean_T emlrtHadParallelError = false;
  emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
  emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
  b_addBiasApplyActivation_numThreads = emlrtAllocRegionTLSs(
      sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel num_threads(b_addBiasApplyActivation_numThreads) private( \
        st, emlrtJBEnviron, v1) firstprivate(emlrtHadParallelError)
  {
    if (setjmp(emlrtJBEnviron) == 0) {
      st.prev = sp;
      st.tls = emlrtAllocTLS((emlrtCTX)sp, omp_get_thread_num());
      st.site = NULL;
      emlrtSetJmpBuf(&st, &emlrtJBEnviron);
    } else {
      emlrtHadParallelError = true;
    }
#pragma omp for nowait
    for (iElem = 0; iElem < 640; iElem++) {
      if (emlrtHadParallelError) {
        continue;
      }
      if (setjmp(emlrtJBEnviron) == 0) {
        v1 = iElem - ((iElem / 64) << 6);
        if ((v1 + 1 < 1) || (v1 + 1 > 64)) {
          emlrtDynamicBoundsCheckR2012b(v1 + 1, 1, 64, &d_emlrtBCI, &st);
        }
      } else {
        emlrtHadParallelError = true;
      }
    }
  }
  emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
  emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
}

/*
 *
 */
void c_addBiasApplyActivation(const emlrtStack *sp)
{
  jmp_buf emlrtJBEnviron;
  jmp_buf *volatile emlrtJBStack;
  emlrtStack st;
  int32_T c_addBiasApplyActivation_numThreads;
  int32_T iElem;
  int32_T varargout_3;
  boolean_T emlrtHadParallelError = false;
  emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
  emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
  c_addBiasApplyActivation_numThreads = emlrtAllocRegionTLSs(
      sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel num_threads(c_addBiasApplyActivation_numThreads) private( \
        st, emlrtJBEnviron, varargout_3) firstprivate(emlrtHadParallelError)
  {
    if (setjmp(emlrtJBEnviron) == 0) {
      st.prev = sp;
      st.tls = emlrtAllocTLS((emlrtCTX)sp, omp_get_thread_num());
      st.site = NULL;
      emlrtSetJmpBuf(&st, &emlrtJBEnviron);
    } else {
      emlrtHadParallelError = true;
    }
#pragma omp for nowait
    for (iElem = 0; iElem < 160; iElem++) {
      if (emlrtHadParallelError) {
        continue;
      }
      if (setjmp(emlrtJBEnviron) == 0) {
        varargout_3 = (iElem - ((iElem >> 4) << 4)) + 1;
        if ((varargout_3 < 1) || (varargout_3 > 16)) {
          emlrtDynamicBoundsCheckR2012b(varargout_3, 1, 16, &c_emlrtBCI, &st);
        }
      } else {
        emlrtHadParallelError = true;
      }
    }
  }
  emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
  emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
}

/*
 *
 */
void d_addBiasApplyActivation(const emlrtStack *sp, real32_T X[10])
{
  jmp_buf emlrtJBEnviron;
  jmp_buf *volatile emlrtJBStack;
  emlrtStack st;
  int32_T d_addBiasApplyActivation_numThreads;
  int32_T iElem;
  boolean_T emlrtHadParallelError = false;
  emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
  emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
  d_addBiasApplyActivation_numThreads = emlrtAllocRegionTLSs(
      sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel num_threads(d_addBiasApplyActivation_numThreads) private( \
        st, emlrtJBEnviron) firstprivate(emlrtHadParallelError)
  {
    if (setjmp(emlrtJBEnviron) == 0) {
      st.prev = sp;
      st.tls = emlrtAllocTLS((emlrtCTX)sp, omp_get_thread_num());
      st.site = NULL;
      emlrtSetJmpBuf(&st, &emlrtJBEnviron);
    } else {
      emlrtHadParallelError = true;
    }
#pragma omp for nowait
    for (iElem = 0; iElem < 10; iElem++) {
      if (emlrtHadParallelError) {
        continue;
      }
      if (setjmp(emlrtJBEnviron) == 0) {
        X[iElem] += 0.130152538F;
      } else {
        emlrtHadParallelError = true;
      }
    }
  }
  emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
  emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
}

/* End of code generation (addBiasApplyActivation.c) */
