/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: internal_softmax.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 01-Sep-2025 14:59:08
 */

/* Include Files */
#include "internal_softmax.h"
#include "elementwiseOperationInPlace.h"
#include "rt_nonfinite.h"
#include "omp.h"
#include "rt_nonfinite.h"
#include <xmmintrin.h>

/* Function Definitions */
/*
 * Arguments    : const float xdata[400]
 *                float ydata[400]
 * Return Type  : void
 */
void iComputeSoftmaxForCpu(const float xdata[400], float ydata[400])
{
  __m128 r;
  __m128 r1;
  double nonChannelSubscriptIndices[3];
  float dataExp[10];
  float f;
  float sumX;
  int i;
  int idx;
  int k;
  int nonChannelDimsProductIdx;
  int v1;
  int vk;
  boolean_T exitg1;
#pragma omp parallel for num_threads(omp_get_max_threads()) private(           \
        sumX, dataExp, nonChannelSubscriptIndices, vk, v1, i, idx, k, f, r,    \
            r1, exitg1)

  for (nonChannelDimsProductIdx = 0; nonChannelDimsProductIdx < 40;
       nonChannelDimsProductIdx++) {
    vk = nonChannelDimsProductIdx / 10;
    v1 = nonChannelDimsProductIdx - vk * 10;
    i = 10 * v1 + 100 * vk;
    sumX = xdata[i];
    if (!rtIsNaNF(sumX)) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k < 11)) {
        if (!rtIsNaNF(xdata[((k + 10 * v1) + 100 * vk) - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx != 0) {
      sumX = xdata[((idx + 10 * v1) + 100 * vk) - 1];
      idx++;
      for (k = idx; k < 11; k++) {
        f = xdata[((k + 10 * v1) + 100 * vk) - 1];
        if (sumX < f) {
          sumX = f;
        }
      }
    }
    r = _mm_set1_ps(sumX);
    _mm_storeu_ps(&dataExp[0], _mm_sub_ps(_mm_loadu_ps(&xdata[i]), r));
    _mm_storeu_ps(&dataExp[4], _mm_sub_ps(_mm_loadu_ps(&xdata[i + 4]), r));
    dataExp[8] = xdata[i + 8] - sumX;
    dataExp[9] = xdata[i + 9] - sumX;
    lambdaForColumnMajorGeneric(dataExp);
    sumX = dataExp[0];
    for (k = 0; k < 9; k++) {
      sumX += dataExp[k + 1];
    }
    r = _mm_loadu_ps(&dataExp[0]);
    r1 = _mm_set1_ps(sumX);
    _mm_storeu_ps(&ydata[i], _mm_div_ps(r, r1));
    r = _mm_loadu_ps(&dataExp[4]);
    _mm_storeu_ps(&ydata[i + 4], _mm_div_ps(r, r1));
    ydata[i + 8] = dataExp[8] / sumX;
    ydata[i + 9] = dataExp[9] / sumX;
  }
}

/*
 * File trailer for internal_softmax.c
 *
 * [EOF]
 */
