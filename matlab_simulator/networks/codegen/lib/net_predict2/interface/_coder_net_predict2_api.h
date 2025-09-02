/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_net_predict2_api.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 01-Sep-2025 14:59:08
 */

#ifndef _CODER_NET_PREDICT2_API_H
#define _CODER_NET_PREDICT2_API_H

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
void net_predict2(real_T in[80], real32_T out[10]);

void net_predict2_api(const mxArray *prhs, const mxArray **plhs);

void net_predict2_atexit(void);

void net_predict2_initialize(void);

void net_predict2_terminate(void);

void net_predict2_xil_shutdown(void);

void net_predict2_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_net_predict2_api.h
 *
 * [EOF]
 */
