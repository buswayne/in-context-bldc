/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: predict.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 16-Feb-2026 17:12:46
 */

/* Include Files */
#include "predict.h"
#include "callPredict.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const float varargin_1_Data[80]
 *                float varargout_1_Data[10]
 * Return Type  : void
 */
void dlnetwork_predict(const float varargin_1_Data[80],
                       float varargout_1_Data[10])
{
  float dataInputsSingle_0_f1[80];
  int b_k;
  int k;
  for (k = 0; k < 8; k++) {
    for (b_k = 0; b_k < 10; b_k++) {
      dataInputsSingle_0_f1[k + (b_k << 3)] = varargin_1_Data[b_k + 10 * k];
    }
  }
  predict(dataInputsSingle_0_f1, varargout_1_Data);
}

/*
 * File trailer for predict.c
 *
 * [EOF]
 */
