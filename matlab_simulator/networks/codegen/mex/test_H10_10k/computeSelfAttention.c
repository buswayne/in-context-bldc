/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeSelfAttention.c
 *
 * Code generation for function 'computeSelfAttention'
 *
 */

/* Include files */
#include "computeSelfAttention.h"
#include "rt_nonfinite.h"
#include "test_H10_10k_data.h"
#include "blas.h"
#include <stddef.h>
#include <xmmintrin.h>

/* Function Definitions */
void iLinearProjectionWithBias(const real32_T in[160],
                               const real32_T weights[256],
                               const real32_T bias[16], real32_T out[160])
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  int32_T i;
  real32_T alpha1;
  real32_T beta1;
  char_T TRANSA1;
  char_T TRANSB1;
  TRANSB1 = 'N';
  TRANSA1 = 'N';
  alpha1 = 1.0F;
  beta1 = 0.0F;
  m_t = (ptrdiff_t)16;
  n_t = (ptrdiff_t)10;
  k_t = (ptrdiff_t)16;
  lda_t = (ptrdiff_t)16;
  ldb_t = (ptrdiff_t)16;
  ldc_t = (ptrdiff_t)16;
  sgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, (real32_T *)&weights[0],
        &lda_t, (real32_T *)&in[0], &ldb_t, &beta1, &out[0], &ldc_t);
  for (i = 0; i < 10; i++) {
    __m128 r;
    int32_T i1;
    i1 = i << 4;
    r = _mm_loadu_ps(&out[i1]);
    _mm_storeu_ps(&out[i1], _mm_add_ps(r, _mm_loadu_ps(&bias[0])));
    r = _mm_loadu_ps(&out[i1 + 4]);
    _mm_storeu_ps(&out[i1 + 4], _mm_add_ps(r, _mm_loadu_ps(&bias[4])));
    r = _mm_loadu_ps(&out[i1 + 8]);
    _mm_storeu_ps(&out[i1 + 8], _mm_add_ps(r, _mm_loadu_ps(&bias[8])));
    r = _mm_loadu_ps(&out[i1 + 12]);
    _mm_storeu_ps(&out[i1 + 12], _mm_add_ps(r, _mm_loadu_ps(&bias[12])));
  }
}

/* End of code generation (computeSelfAttention.c) */
