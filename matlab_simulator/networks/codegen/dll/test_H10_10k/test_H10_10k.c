/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: test_H10_10k.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Jan-2026 10:52:33
 */

/* Include Files */
#include "test_H10_10k.h"
#include "predict.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * A persistent object mynet is used to load the series network object.
 *  At the first call to this function, the persistent object is constructed and
 *  setup. When the function is called subsequent times, the same object is
 * reused to call predict on inputs, thus avoiding reconstructing and reloading
 * the network object.
 *
 * Arguments    : const double in[80]
 *                float out[10]
 * Return Type  : void
 */
void test_H10_10k(const double in[80], float out[10])
{
  float dl_in_Data[80];
  int i;
  /*  pass in input    */
  for (i = 0; i < 80; i++) {
    dl_in_Data[i] = (float)in[i];
  }
  dlnetwork_predict(dl_in_Data, out);
}

/*
 * File trailer for test_H10_10k.c
 *
 * [EOF]
 */
