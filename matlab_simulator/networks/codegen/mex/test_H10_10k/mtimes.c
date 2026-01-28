/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mtimes.c
 *
 * Code generation for function 'mtimes'
 *
 */

/* Include files */
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "test_H10_10k_data.h"
#include "blas.h"
#include <stddef.h>

/* Function Definitions */
void mtimes(const real32_T A[1024], const real32_T B[640], real32_T C[160])
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
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
  k_t = (ptrdiff_t)64;
  lda_t = (ptrdiff_t)16;
  ldb_t = (ptrdiff_t)64;
  ldc_t = (ptrdiff_t)16;
  sgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, (real32_T *)&A[0],
        &lda_t, (real32_T *)&B[0], &ldb_t, &beta1, &C[0], &ldc_t);
}

/* End of code generation (mtimes.c) */
