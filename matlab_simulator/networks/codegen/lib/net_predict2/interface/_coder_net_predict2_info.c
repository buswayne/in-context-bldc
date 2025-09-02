/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_net_predict2_info.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 01-Sep-2025 14:59:08
 */

/* Include Files */
#include "_coder_net_predict2_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : const mxArray *
 */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[4] = {
      "789c6360f4f465646060e0638000033608cd0be50b4069260654802ecf88838601560616"
      "147d30f97e289d9c9f57925a5102e1e425e6a6c275a6e4e766e625e6"
      "95845416a43214a516e7e794a5a68065d2327352433273538391397e205eae1b92149c03"
      "9202b19d335293b3834b73198a328a112ecc41e6c0c32301877f5908",
      "84073a400f0f747523c5be1364da07333f8d807d30f9e8d05867ab98d0e2d4a2e218634b"
      "6313831897fce4d2dcd4bc92e218f7cc128fd2a498cc3c5d68aad34d"
      "ca49498ec94d2cc9494c8a2fcecc2dcd492cc92f8ac94b2d29cf2fca2e8ec94b2988cfcb"
      "cf2c4e8dcf3034883731c8f63034f0d003aa271c8efc44fa0b57bee1",
      "65e000d3fa8c77c052f4b2ef9c60f9367ada070303655f050ef3884d97a238ec13409337"
      "f52e774b0bf50ff00a770e350a4bae3471728b0844724700017b08b9"
      "8301079fd6e6030023095e5c",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 1608U, &nameCaptureInfo);
  return nameCaptureInfo;
}

/*
 * Arguments    : void
 * Return Type  : mxArray *
 */
mxArray *emlrtMexFcnProperties(void)
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *propFieldName[9] = {"Version",
                                    "ResolvedFunctions",
                                    "Checksum",
                                    "EntryPoints",
                                    "CoverageInfo",
                                    "IsPolymorphic",
                                    "PropertyList",
                                    "UUID",
                                    "ClassEntryPointIsHandle"};
  const char_T *epFieldName[8] = {
      "QualifiedName",    "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "ResolvedFilePath", "TimeStamp",      "Constructor",     "Visible"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 1);
  emlrtSetField(xEntryPoints, 0, "QualifiedName",
                emlrtMxCreateString("net_predict2"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "ResolvedFilePath",
      emlrtMxCreateString("C:\\Users\\39340\\Documents\\GitHub\\in-context-"
                          "bldc\\matlab_simulator\\networks\\net_predict2.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739861.60074074077));
  emlrtSetField(xEntryPoints, 0, "Constructor",
                emlrtMxCreateLogicalScalar(false));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("24.2.0.2923080 (R2024b) Update 6"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("5KwFfUOPJWCU2Vcy4BFXQ"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

/*
 * File trailer for _coder_net_predict2_info.c
 *
 * [EOF]
 */
