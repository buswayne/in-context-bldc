/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_ndp_random_correction_long_info.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 16-Feb-2026 17:15:06
 */

/* Include Files */
#include "_coder_ndp_random_correction_long_info.h"
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
      "789c6360f4f465646060e06380000b3608cd0be50b4069260654802ecf88838601560616"
      "147d30f97e289d9c9f57925a5102e1e425e6a6c275a6e4e766e625e6"
      "95845416a43214a516e7e794a5a68065d2327352433273538391397e205eae1b92149c03"
      "9202b19d335293b3834b73198a328a112ecc41e6c0c32301877f5908",
      "84073a400f0f747523c5be0b64da0733bf80807d30f9e8d05867ab98d0e2d4a2e218634b"
      "6313831897fce4d2dcd4bc92e218f7cc128fd2a498cc3c5d68aad34d"
      "ca49498ec94d2cc9494c8a2fcecc2dcd492cc92f8ac94b2d29cf2fca2e8ec94b29882f4a"
      "cc03a6c4f8e4fca2a2d4e492ccfcbcf89cfcbcf4780f43030f3da04e",
      "42e1c94fa4ff70e51f5e060e301d37ff0e588a5ef6ad5b79c28c9ef6c1c040d95781c33c"
      "62d3a7180efb04d0e42b9242c2dc724b2b8a9c4a2c0d922bb3b39c22"
      "1db33d10ee0820600f217730e0e0d3da7c009b766606",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 1616U, &nameCaptureInfo);
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
                emlrtMxCreateString("ndp_random_correction_long"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "ResolvedFilePath",
      emlrtMxCreateString(
          "C:\\Users\\39340\\Documents\\GitHub\\in-context-bldc\\matlab_"
          "simulator\\networks\\ndp_random_correction_long.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(740029.714212963));
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
                emlrtMxCreateString("xbTVFmuxrBt90cykjBYAkH"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

/*
 * File trailer for _coder_ndp_random_correction_long_info.c
 *
 * [EOF]
 */
