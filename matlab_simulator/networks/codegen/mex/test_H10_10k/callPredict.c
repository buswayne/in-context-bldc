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
#include "rt_nonfinite.h"
#include "test_H10_10k_data.h"

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
      -0.00621177303F,  -0.0078556817F,  0.00168816932F,   0.0231609661F,
      0.0141616799F,    -0.0180240255F,  0.0237301F,       0.0273409616F,
      0.00246629911F,   0.0027800377F,   0.00275443657F,   -0.0170256235F,
      0.0150080537F,    -0.013826686F,   -0.00584407616F,  -0.00731953513F,
      -0.00300306268F,  -0.01264768F,    0.0246155802F,    0.00611271849F,
      0.0199723933F,    -0.00281618745F, 0.0259348135F,    0.00786392204F,
      0.0105156135F,    0.00870242808F,  0.0161363743F,    0.0271770228F,
      0.0137574635F,    0.0199607629F,   0.00339844218F,   -0.00572683662F,
      -0.00478403131F,  0.000714837399F, -0.00248916028F,  -0.00957409479F,
      -0.00824581087F,  -0.00328339962F, 0.00456249295F,   -0.0135482112F,
      -0.0114165368F,   0.0147691816F,   0.00484670559F,   0.00514281075F,
      0.00734817097F,   -0.00238585426F, -0.0017609864F,   0.00518125F,
      -0.00803730171F,  0.0101163769F,   0.0129856905F,    0.0054792692F,
      -0.00325162546F,  -0.0026916F,     0.0108314808F,    -0.00286377035F,
      0.00884636492F,   0.0126983104F,   -0.00447104918F,  0.00730620697F,
      0.0181965027F,    -0.0280071311F,  -0.0179116465F,   0.000282937341F,
      0.00221027294F,   -0.0260708146F,  0.010666199F,     -0.00741177471F,
      0.00938648451F,   -0.00761424564F, -0.0100942058F,   0.015515117F,
      -0.00432512909F,  0.00457551656F,  -0.015101321F,    -0.0086267069F,
      -0.00277154497F,  0.00681713084F,  -0.000349196023F, 0.0112361889F,
      -0.000468108774F, 0.0105277924F,   0.0254849605F,    -0.0310788155F,
      0.0076212748F,    0.0311630033F,   -0.0248248708F,   -0.00642611319F,
      -0.0154891387F,   0.0171150975F,   -0.00579433329F,  -0.019760523F,
      0.00973573F,      0.0125476746F,   -0.00226118439F,  0.0267981049F,
      -0.00903790165F,  -0.0205805786F,  -0.000876473961F, -0.0233196579F,
      0.0101971468F,    0.000498373294F, -0.0187160168F,   0.0126761328F,
      -0.0360647589F,   0.00664105872F,  -0.0107307574F,   0.00164106896F,
      -0.00352412881F,  0.0376642346F,   -0.00243915501F,  -0.0129099404F,
      -0.00319081871F,  0.00103429647F,  0.0192497987F,    -0.000413195754F,
      0.0023601905F,    0.00476863701F,  -0.00741176074F,  0.00674091512F,
      -0.0150139416F,   0.0135969007F,   -0.00138997519F,  0.023443535F,
      0.00840864237F,   0.0143024633F,   -0.00209800689F,  0.00793108065F,
      0.000736215559F,  -0.00867937133F, -0.00860684458F,  -0.0145283779F,
      -0.0118512353F,   -0.0243453644F,  0.00216272753F,   -0.00616765255F,
      -0.0134208715F,   0.0191374198F,   -0.016964497F,    -0.00594104407F,
      0.00391214667F,   -0.00602686126F, 0.00160902308F,   0.0110580092F,
      0.00552010071F,   -0.0193126556F,  -0.000639663311F, -0.00813368801F,
      -0.0087374365F,   -0.0265788138F,  0.00737520633F,   -9.26771245E-5F,
      -0.01144276F,     0.00550933648F,  -0.0201253872F,   -0.018134702F,
      -0.000105732412F, -0.00548596494F, 0.00704508508F,   0.0215185657F};
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
