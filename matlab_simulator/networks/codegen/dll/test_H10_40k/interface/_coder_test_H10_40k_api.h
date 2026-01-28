/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_test_H10_40k_api.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Jan-2026 10:49:46
 */

#ifndef _CODER_TEST_H10_40K_API_H
#define _CODER_TEST_H10_40K_API_H

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
void test_H10_40k(real_T in[80], real32_T out[10]);

void test_H10_40k_api(const mxArray *prhs, const mxArray **plhs);

void test_H10_40k_atexit(void);

void test_H10_40k_initialize(void);

void test_H10_40k_terminate(void);

void test_H10_40k_xil_shutdown(void);

void test_H10_40k_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_test_H10_40k_api.h
 *
 * [EOF]
 */
