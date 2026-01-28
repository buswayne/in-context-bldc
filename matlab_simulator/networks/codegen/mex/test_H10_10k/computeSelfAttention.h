/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeSelfAttention.h
 *
 * Code generation for function 'computeSelfAttention'
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
void iLinearProjectionWithBias(const real32_T in[160],
                               const real32_T weights[256],
                               const real32_T bias[16], real32_T out[160]);

/* End of code generation (computeSelfAttention.h) */
