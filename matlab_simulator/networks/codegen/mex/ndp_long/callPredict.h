/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * callPredict.h
 *
 * Code generation for function 'callPredict'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "covrt.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void predict(const emlrtStack *sp, const real32_T inputsT_0_f1[80],
             real32_T outputs_0_f1[10]);

/* End of code generation (callPredict.h) */
