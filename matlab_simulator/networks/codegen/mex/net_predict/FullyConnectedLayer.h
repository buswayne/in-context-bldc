/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * FullyConnectedLayer.h
 *
 * Code generation for function 'FullyConnectedLayer'
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
void FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[80],
                                 real32_T Z[160]);

void b_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[160],
                                   real32_T Z[640]);

void c_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[640],
                                   real32_T Z[160]);

void d_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[160],
                                   real32_T Z[640]);

void e_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[640],
                                   real32_T Z[160]);

void f_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[160],
                                   real32_T Z[640]);

void g_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[640],
                                   real32_T Z[160]);

void h_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[160],
                                   real32_T Z[640]);

void i_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[640],
                                   real32_T Z[160]);

void j_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[160],
                                   real32_T Z[640]);

void k_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[640],
                                   real32_T Z[160]);

void l_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[160],
                                   real32_T Z[640]);

void m_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[640],
                                   real32_T Z[160]);

void n_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[160],
                                   real32_T Z[640]);

void o_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[640],
                                   real32_T Z[160]);

void p_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[160],
                                   real32_T Z[640]);

void q_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[640],
                                   real32_T Z[160]);

void r_FullyConnectedLayer_predict(const emlrtStack *sp, const real32_T X[160],
                                   real32_T Z[10]);

/* End of code generation (FullyConnectedLayer.h) */
