/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * addBiasApplyActivation.h
 *
 * Code generation for function 'addBiasApplyActivation'
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
void addBiasApplyActivation(const emlrtStack *sp, real32_T X[160],
                            const real32_T bias[16]);

void b_addBiasApplyActivation(const emlrtStack *sp);

void c_addBiasApplyActivation(const emlrtStack *sp);

void d_addBiasApplyActivation(const emlrtStack *sp, real32_T X[10]);

/* End of code generation (addBiasApplyActivation.h) */
