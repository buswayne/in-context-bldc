/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: binaryElementwise.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 01-Sep-2025 14:59:08
 */

/* Include Files */
#include "binaryElementwise.h"
#include "rt_nonfinite.h"
#include "omp.h"
#include <xmmintrin.h>

/* Function Declarations */
static void elementwise_addition(const float *inputTensor0,
                                 const float *inputTensor1,
                                 float *outputTensor);

/* Function Definitions */
/*
 * Arguments    : const float *inputTensor0
 *                const float *inputTensor1
 *                float *outputTensor
 * Return Type  : void
 */
static void elementwise_addition(const float *inputTensor0,
                                 const float *inputTensor1, float *outputTensor)
{
  __m128 regSimd4_0;
  __m128 regSimd4_1;
  int regSimd4_0_tmp;
  int simdBlockIdx;
#pragma omp parallel for num_threads(omp_get_max_threads()) private(           \
        regSimd4_0_tmp, regSimd4_0, regSimd4_1)

  for (simdBlockIdx = 0; simdBlockIdx < 40; simdBlockIdx++) {
    regSimd4_0_tmp = simdBlockIdx << 2;
    regSimd4_0 = _mm_loadu_ps(&inputTensor0[regSimd4_0_tmp]);
    regSimd4_1 = _mm_loadu_ps(&inputTensor1[regSimd4_0_tmp]);
    regSimd4_0 = _mm_add_ps(regSimd4_1, regSimd4_0);
    _mm_storeu_ps(&outputTensor[regSimd4_0_tmp], regSimd4_0);
  }
}

/*
 * Arguments    : const float input1[160]
 *                const float input2[160]
 *                float Z[160]
 * Return Type  : void
 */
void binaryElementwise(const float input1[160], const float input2[160],
                       float Z[160])
{
  elementwise_addition(&input1[0], &input2[0], &Z[0]);
}

/*
 * File trailer for binaryElementwise.c
 *
 * [EOF]
 */
