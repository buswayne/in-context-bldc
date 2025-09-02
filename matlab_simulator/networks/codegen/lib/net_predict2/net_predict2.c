/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: net_predict2.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 01-Sep-2025 14:59:08
 */

/* Include Files */
#include "net_predict2.h"
#include "net_predict2_data.h"
#include "net_predict2_initialize.h"
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
void net_predict2(const double in[80], float out[10])
{
  float dl_in_Data[80];
  int i;
  if (!isInitialized_net_predict2) {
    net_predict2_initialize();
  }
  /*  pass in input    */
  for (i = 0; i < 80; i++) {
    dl_in_Data[i] = (float)in[i];
  }
  dlnetwork_predict(dl_in_Data, out);
}

/*
 * File trailer for net_predict2.c
 *
 * [EOF]
 */
