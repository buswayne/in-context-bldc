/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * callPredict.c
 *
 * Code generation for function 'callPredict'
 *
 */

/* Include files */
#include "callPredict.h"
#include "FullyConnectedLayer.h"
#include "GELULayer.h"
#include "LayerNormalizationLayer.h"
#include "SelfAttentionLayer.h"
#include "binaryElementwise.h"
#include "ndp_long_data.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo b_emlrtRSI = {
    1,                    /* lineNo */
    "profileRegionStart", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\gpucoder\\gpucoder\\+coder\\+"
    "gpu\\+internal\\profileRegionStart.p" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    1,                              /* lineNo */
    "dummyCodeRegionSim/regionEnd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\gpucoder\\gpucoder\\+coder\\+"
    "gpu\\+internal\\dummyCodeRegionSim.p" /* pathName */
};

static emlrtRSInfo h_emlrtRSI = {
    1,                       /* lineNo */
    "dlnetwork/callPredict", /* fcnName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\+ctarget\\@dlnetwork\\callPredict.p" /* pathName */
};

static emlrtRSInfo i_emlrtRSI = {
    90,               /* lineNo */
    "callActivation", /* fcnName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\+ctarget\\+networkUtils\\callActivation.m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI = {
    155,              /* lineNo */
    "getActivations", /* fcnName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\+ctarget\\+networkUtils\\getActivations.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    117,              /* lineNo */
    "getActivations", /* fcnName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\+ctarget\\+networkUtils\\getActivations.m" /* pathName */
};

static emlrtRSInfo l_emlrtRSI = {
    281,                       /* lineNo */
    "iInvokeLayerPredictCall", /* fcnName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\+ctarget\\+networkUtils\\getActivations.m" /* pathName */
};

static emlrtRSInfo m_emlrtRSI = {
    285,                       /* lineNo */
    "iInvokeLayerPredictCall", /* fcnName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\+ctarget\\+networkUtils\\getActivations.m" /* pathName */
};

static emlrtRSInfo n_emlrtRSI = {
    52,                  /* lineNo */
    "invokePredictCall", /* fcnName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\+ctarget\\+networkUtils\\invokePredictCall.m" /* pathName */
};

static emlrtRSInfo t_emlrtRSI = {
    46,                  /* lineNo */
    "invokePredictCall", /* fcnName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\+ctarget\\+networkUtils\\invokePredictCall.m" /* pathName */
};

static emlrtRSInfo u_emlrtRSI = {
    1,                                      /* lineNo */
    "coderNetworkUtils/customLayerPredict", /* fcnName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\@coderNetworkUtils\\customLayerPredict.p" /* pathName */
};

static emlrtRSInfo v_emlrtRSI = {
    1,                       /* lineNo */
    "iCallLayerPredictImpl", /* fcnName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\@coderNetworkUtils\\customLayerPredict.p" /* pathName */
};

static emlrtRSInfo xb_emlrtRSI = {
    1,                    /* lineNo */
    "elementwiseForward", /* fcnName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\+layer\\+elementwiseUtils\\elementwiseForward.p" /* pathName */
};

/* Function Definitions */
void predict(const emlrtStack *sp, const real32_T inputsT_0_f1[80],
             real32_T outputs_0_f1[10])
{
  static const real32_T fv1[160] = {
      -0.00300077489F,  -0.00725745689F,  0.00407219911F,   0.0221765749F,
      0.0157630127F,    -0.0175269172F,   0.0259700213F,    0.0261996351F,
      0.00214956375F,   0.000425556267F,  -0.000299139647F, -0.0141432164F,
      0.0124897286F,    -0.0121329427F,   -0.0071991249F,   -0.00796768442F,
      -0.00307040568F,  -0.0120417941F,   0.0233082511F,    0.00734820869F,
      0.0214549322F,    -0.0037159503F,   0.0246668905F,    0.00948985666F,
      0.0107822558F,    0.00989884511F,   0.0154500427F,    0.0214817226F,
      0.0133505184F,    0.0168522149F,    0.00355491671F,   -0.00640228344F,
      -0.00565824797F,  0.000715826347F,  0.00352396909F,   -0.0131718861F,
      -0.00479462277F,  -0.00379241654F,  0.00438228576F,   -0.0171688385F,
      -0.00930788554F,  0.0147014763F,    0.00594226504F,   0.00619169977F,
      0.00714677339F,   -0.00547950342F,  -0.00336212642F,  0.00487046828F,
      -0.00780871F,     0.00878797F,      0.0109474137F,    0.00294210133F,
      -0.00327685219F,  -0.00038499973F,  0.0106267408F,    -0.00553735113F,
      0.00855660439F,   0.0132372715F,    -0.00269020023F,  0.0122409118F,
      0.018135095F,     -0.0238096137F,   -0.0176861212F,   0.001733104F,
      -0.00205370015F,  -0.024677325F,    0.0116961878F,    -0.00816944148F,
      0.00958715472F,   -0.00647453452F,  -0.00796011183F,  0.0108519122F,
      -0.004125698F,    0.00382481469F,   -0.0137289995F,   -0.0042146272F,
      -0.00136225484F,  0.0092080459F,    -0.000490818638F, 0.0095153749F,
      0.00145379989F,   0.0107188616F,    0.0234756079F,    -0.0289263558F,
      0.00927498471F,   0.0259255581F,    -0.0225415863F,   -0.00594113069F,
      -0.0174832214F,   0.0175514612F,    -0.0124115851F,   -0.0143531077F,
      0.0106266253F,    0.0124193057F,    -0.00221430976F,  0.0253282767F,
      -0.0111127766F,   -0.02007268F,     0.00427300716F,   -0.0228208303F,
      0.00880744494F,   -0.000162796277F, -0.0186775811F,   0.0105987424F,
      -0.0347620882F,   0.00540036522F,   -0.0136590209F,   0.00271745096F,
      -0.0045547504F,   0.0346669443F,    -0.00199706363F,  -0.0113327773F,
      -0.00222664699F,  -7.34744463E-5F,  0.0124669392F,    -0.00149960385F,
      0.0033708855F,    0.0050375131F,    -0.00814656541F,  0.00738996919F,
      -0.015574269F,    0.0160742905F,    -0.00242470833F,  0.0185952708F,
      0.00653190864F,   0.0154668288F,    -0.00118757517F,  0.00903768F,
      -0.00142302946F,  -0.00929268636F,  -0.0105232624F,   -0.0132560991F,
      -0.0113425255F,   -0.0173655618F,   0.000831450394F,  -0.00491146976F,
      -0.014339528F,    0.0155873187F,    -0.0145202978F,   -0.00840231497F,
      0.00539578311F,   -0.00412649754F,  0.00234981906F,   0.0130935973F,
      0.00421748869F,   -0.0197115522F,   -0.00762274675F,  -0.00699040527F,
      -0.00775780203F,  -0.025027791F,    0.0025988752F,    0.00161223928F,
      -0.0111175291F,   0.00808865298F,   -0.0201982744F,   -0.0163782053F,
      -0.000333212607F, -0.00388261792F,  0.00695447158F,   0.0212562215F};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  real32_T outT_f10_0_f1[640];
  real32_T fv[160];
  real32_T outT_f3_0_f1[160];
  real32_T outT_f8_0_f1[160];
  real32_T out_0_Data[160];
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &h_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  b_st.site = &i_emlrtRSI;
  c_st.site = &k_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  FullyConnectedLayer_predict(&e_st, inputsT_0_f1, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  binaryElementwise(&d_st, fv1, outT_f3_0_f1, fv);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  LayerNormalizationLayer_predict(fv, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  g_st.site = &v_emlrtRSI;
  SelfAttentionLayer_predict(&g_st, out_0_Data, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  binaryElementwise(&d_st, outT_f3_0_f1, fv, outT_f8_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  b_LayerNormalizationLayer_predi(outT_f8_0_f1, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  b_FullyConnectedLayer_predict(&e_st, out_0_Data, outT_f10_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  GELULayer_predict(outT_f10_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  c_FullyConnectedLayer_predict(&e_st, outT_f10_0_f1, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  binaryElementwise(&d_st, outT_f3_0_f1, outT_f8_0_f1, fv);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  c_LayerNormalizationLayer_predi(fv, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  g_st.site = &v_emlrtRSI;
  b_SelfAttentionLayer_predict(&g_st, out_0_Data, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  binaryElementwise(&d_st, outT_f3_0_f1, fv, outT_f8_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  d_LayerNormalizationLayer_predi(outT_f8_0_f1, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  d_FullyConnectedLayer_predict(&e_st, out_0_Data, outT_f10_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  GELULayer_predict(outT_f10_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  e_FullyConnectedLayer_predict(&e_st, outT_f10_0_f1, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  binaryElementwise(&d_st, outT_f3_0_f1, outT_f8_0_f1, fv);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  e_LayerNormalizationLayer_predi(fv, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  g_st.site = &v_emlrtRSI;
  c_SelfAttentionLayer_predict(&g_st, out_0_Data, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  binaryElementwise(&d_st, outT_f3_0_f1, fv, outT_f8_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  f_LayerNormalizationLayer_predi(outT_f8_0_f1, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  f_FullyConnectedLayer_predict(&e_st, out_0_Data, outT_f10_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  GELULayer_predict(outT_f10_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  g_FullyConnectedLayer_predict(&e_st, outT_f10_0_f1, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  binaryElementwise(&d_st, outT_f3_0_f1, outT_f8_0_f1, fv);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  g_LayerNormalizationLayer_predi(fv, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  g_st.site = &v_emlrtRSI;
  d_SelfAttentionLayer_predict(&g_st, out_0_Data, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  binaryElementwise(&d_st, outT_f3_0_f1, fv, outT_f8_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  h_LayerNormalizationLayer_predi(outT_f8_0_f1, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  h_FullyConnectedLayer_predict(&e_st, out_0_Data, outT_f10_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  GELULayer_predict(outT_f10_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  i_FullyConnectedLayer_predict(&e_st, outT_f10_0_f1, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  binaryElementwise(&d_st, outT_f3_0_f1, outT_f8_0_f1, fv);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  i_LayerNormalizationLayer_predi(fv, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  g_st.site = &v_emlrtRSI;
  e_SelfAttentionLayer_predict(&g_st, out_0_Data, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  binaryElementwise(&d_st, outT_f3_0_f1, fv, outT_f8_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  j_LayerNormalizationLayer_predi(outT_f8_0_f1, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  j_FullyConnectedLayer_predict(&e_st, out_0_Data, outT_f10_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  GELULayer_predict(outT_f10_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  k_FullyConnectedLayer_predict(&e_st, outT_f10_0_f1, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  binaryElementwise(&d_st, outT_f3_0_f1, outT_f8_0_f1, fv);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  k_LayerNormalizationLayer_predi(fv, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  g_st.site = &v_emlrtRSI;
  f_SelfAttentionLayer_predict(&g_st, out_0_Data, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  binaryElementwise(&d_st, outT_f3_0_f1, fv, outT_f8_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  l_LayerNormalizationLayer_predi(outT_f8_0_f1, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  l_FullyConnectedLayer_predict(&e_st, out_0_Data, outT_f10_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  GELULayer_predict(outT_f10_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  m_FullyConnectedLayer_predict(&e_st, outT_f10_0_f1, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  binaryElementwise(&d_st, outT_f3_0_f1, outT_f8_0_f1, fv);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  m_LayerNormalizationLayer_predi(fv, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  g_st.site = &v_emlrtRSI;
  g_SelfAttentionLayer_predict(&g_st, out_0_Data, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  binaryElementwise(&d_st, outT_f3_0_f1, fv, outT_f8_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  n_LayerNormalizationLayer_predi(outT_f8_0_f1, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  n_FullyConnectedLayer_predict(&e_st, out_0_Data, outT_f10_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  GELULayer_predict(outT_f10_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  o_FullyConnectedLayer_predict(&e_st, outT_f10_0_f1, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  binaryElementwise(&d_st, outT_f3_0_f1, outT_f8_0_f1, fv);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  o_LayerNormalizationLayer_predi(fv, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  g_st.site = &v_emlrtRSI;
  h_SelfAttentionLayer_predict(&g_st, out_0_Data, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  binaryElementwise(&d_st, outT_f3_0_f1, fv, outT_f8_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  p_LayerNormalizationLayer_predi(outT_f8_0_f1, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  p_FullyConnectedLayer_predict(&e_st, out_0_Data, outT_f10_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  GELULayer_predict(outT_f10_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  q_FullyConnectedLayer_predict(&e_st, outT_f10_0_f1, outT_f3_0_f1);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  f_st.site = &u_emlrtRSI;
  g_st.site = &xb_emlrtRSI;
  binaryElementwise(&g_st, outT_f3_0_f1, outT_f8_0_f1, fv);
  q_LayerNormalizationLayer_predi(fv, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  r_FullyConnectedLayer_predict(out_0_Data, outputs_0_f1);
}

/* End of code generation (callPredict.c) */
