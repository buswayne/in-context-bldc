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

static emlrtRSInfo yb_emlrtRSI = {
    1,                    /* lineNo */
    "elementwiseForward", /* fcnName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\+layer\\+elementwiseUtils\\elementwiseForward.p" /* pathName */
};

/* Function Definitions */
/*
 *
 */
void predict(const emlrtStack *sp, const real32_T inputsT_0_f1[80],
             real32_T outputs_0_f1[10])
{
  static const real32_T fv1[160] = {
      -0.00273892237F,  -0.00726709468F,  0.00486792345F,   0.0220692493F,
      0.0159275886F,    -0.0170771684F,   0.0262280125F,    0.025803335F,
      0.00225513219F,   -0.000182079864F, -0.000273592217F, -0.013958253F,
      0.0126012284F,    -0.0117478482F,   -0.00749261631F,  -0.00801250525F,
      -0.00294444733F,  -0.0117891906F,   0.0230069552F,    0.00737451715F,
      0.0215394665F,    -0.0040928456F,   0.024546586F,     0.0103095481F,
      0.0106416885F,    0.0100915516F,    0.0148768229F,    0.0207487848F,
      0.0130182188F,    0.0161280185F,    0.00356330629F,   -0.00634297F,
      -0.00591473281F,  0.000883533678F,  0.00459496118F,   -0.0135226203F,
      -0.00398990419F,  -0.00352185196F,  0.00473504793F,   -0.0174885783F,
      -0.00862672366F,  0.0142515991F,    0.0054891808F,    0.00560896704F,
      0.00712934742F,   -0.00569083402F,  -0.00378788F,     0.00437549734F,
      -0.00748420414F,  0.0085740136F,    0.010391295F,     0.00288590253F,
      -0.00365893426F,  -0.000478109461F, 0.010474544F,     -0.00549192913F,
      0.00826597866F,   0.013511627F,     -0.00200679037F,  0.013496113F,
      0.0175634641F,    -0.0231842175F,   -0.0173789766F,   0.00210572174F,
      -0.0029108203F,   -0.0244760178F,   0.0120538948F,    -0.00825507659F,
      0.00956626888F,   -0.00620103953F,  -0.00765977474F,  0.0100755692F,
      -0.00402898341F,  0.00423629349F,   -0.0130367456F,   -0.00253596599F,
      -0.00135205803F,  0.010155513F,     -0.000746073318F, 0.00889365F,
      0.00126675807F,   0.0107474606F,    0.0236054901F,    -0.0287430845F,
      0.0089905709F,    0.0247126073F,    -0.0217819475F,   -0.00557280704F,
      -0.0179081429F,   0.0176420379F,    -0.0133322487F,   -0.0128848972F,
      0.0106226169F,    0.0135919712F,    -0.00238236925F,  0.0254524332F,
      -0.0117413243F,   -0.0199758317F,   0.00511269644F,   -0.0222128853F,
      0.00831176434F,   -0.00111209461F,  -0.0186658986F,   0.0102145523F,
      -0.0343410149F,   0.00515209185F,   -0.0142307077F,   0.00264087762F,
      -0.00472560525F,  0.0341134779F,    -0.00217199116F,  -0.010417372F,
      -0.00187646179F,  -0.000243263814F, 0.0111548807F,    -0.00194177672F,
      0.00354651897F,   0.0049308734F,    -0.00810378883F,  0.00723232748F,
      -0.0156695507F,   0.0163987968F,    -0.00289155892F,  0.0165941063F,
      0.00674269721F,   0.01605157F,      -0.000819118635F, 0.00908862334F,
      -0.000805213116F, -0.00944848359F,  -0.0112353F,      -0.0124690309F,
      -0.0109023564F,   -0.0164392143F,   -0.00036153206F,  -0.00518650748F,
      -0.0144831305F,   0.0152444318F,    -0.0137170423F,   -0.00842393F,
      0.00547658419F,   -0.00502365036F,  0.0026217415F,    0.0134304836F,
      0.00384908845F,   -0.0198208708F,   -0.00875786413F,  -0.00739736715F,
      -0.00757232215F,  -0.0236108F,      0.00162579527F,   0.00181441626F,
      -0.0111317839F,   0.00864245184F,   -0.0196031071F,   -0.0158734471F,
      0.000227921264F,  -0.00378972758F,  0.0069398745F,    0.0208530352F};
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
  d_st.site = &yb_emlrtRSI;
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
  d_st.site = &yb_emlrtRSI;
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
  g_st.site = &v_emlrtRSI;
  GELULayer_predict(&g_st, outT_f10_0_f1);
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
  d_st.site = &yb_emlrtRSI;
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
  d_st.site = &yb_emlrtRSI;
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
  g_st.site = &v_emlrtRSI;
  GELULayer_predict(&g_st, outT_f10_0_f1);
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
  d_st.site = &yb_emlrtRSI;
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
  d_st.site = &yb_emlrtRSI;
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
  g_st.site = &v_emlrtRSI;
  GELULayer_predict(&g_st, outT_f10_0_f1);
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
  d_st.site = &yb_emlrtRSI;
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
  d_st.site = &yb_emlrtRSI;
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
  g_st.site = &v_emlrtRSI;
  GELULayer_predict(&g_st, outT_f10_0_f1);
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
  d_st.site = &yb_emlrtRSI;
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
  d_st.site = &yb_emlrtRSI;
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
  g_st.site = &v_emlrtRSI;
  GELULayer_predict(&g_st, outT_f10_0_f1);
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
  d_st.site = &yb_emlrtRSI;
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
  d_st.site = &yb_emlrtRSI;
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
  g_st.site = &v_emlrtRSI;
  GELULayer_predict(&g_st, outT_f10_0_f1);
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
  d_st.site = &yb_emlrtRSI;
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
  d_st.site = &yb_emlrtRSI;
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
  g_st.site = &v_emlrtRSI;
  GELULayer_predict(&g_st, outT_f10_0_f1);
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
  d_st.site = &yb_emlrtRSI;
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
  d_st.site = &yb_emlrtRSI;
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
  g_st.site = &v_emlrtRSI;
  GELULayer_predict(&g_st, outT_f10_0_f1);
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
  g_st.site = &yb_emlrtRSI;
  binaryElementwise(&g_st, outT_f3_0_f1, outT_f8_0_f1, fv);
  q_LayerNormalizationLayer_predi(fv, out_0_Data);
  c_st.site = &j_emlrtRSI;
  d_st.site = &l_emlrtRSI;
  e_st.site = &b_emlrtRSI;
  f_st.site = &c_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  e_st.site = &n_emlrtRSI;
  r_FullyConnectedLayer_predict(&e_st, out_0_Data, outputs_0_f1);
}

/* End of code generation (callPredict.c) */
