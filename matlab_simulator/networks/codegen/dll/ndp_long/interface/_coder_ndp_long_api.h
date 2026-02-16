/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_ndp_long_api.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 16-Feb-2026 17:11:19
 */

#ifndef _CODER_NDP_LONG_API_H
#define _CODER_NDP_LONG_API_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void ndp_long(real_T in[80], real32_T out[10]);

void ndp_long_api(const mxArray *prhs, const mxArray **plhs);

void ndp_long_atexit(void);

void ndp_long_initialize(void);

void ndp_long_terminate(void);

void ndp_long_xil_shutdown(void);

void ndp_long_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_ndp_long_api.h
 *
 * [EOF]
 */
