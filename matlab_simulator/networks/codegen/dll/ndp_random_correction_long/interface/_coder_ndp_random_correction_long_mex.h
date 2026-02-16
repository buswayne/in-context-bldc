/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_ndp_random_correction_long_mex.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 16-Feb-2026 17:15:06
 */

#ifndef _CODER_NDP_RANDOM_CORRECTION_LONG_MEX_H
#define _CODER_NDP_RANDOM_CORRECTION_LONG_MEX_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

void unsafe_ndp_random_correction_long_mexFunction(int32_T nlhs,
                                                   mxArray *plhs[1],
                                                   int32_T nrhs,
                                                   const mxArray *prhs[1]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_ndp_random_correction_long_mex.h
 *
 * [EOF]
 */
