/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * attentionForward.h
 *
 * Code generation for function 'attentionForward'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void attentionForward(const emlrtStack *sp, const real32_T Q[160],
                      const real32_T K[160], const real32_T V[160],
                      real32_T X[160]);

/* End of code generation (attentionForward.h) */
