/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: net_predict2_initialize.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 01-Sep-2025 14:59:08
 */

/* Include Files */
#include "net_predict2_initialize.h"
#include "net_predict2_data.h"
#include "rt_nonfinite.h"
#include "omp.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void net_predict2_initialize(void)
{
  omp_init_nest_lock(&net_predict2_nestLockGlobal);
  isInitialized_net_predict2 = true;
}

/*
 * File trailer for net_predict2_initialize.c
 *
 * [EOF]
 */
