/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_net_predict_info.c
 *
 * Code generation for function 'net_predict'
 *
 */

/* Include files */
#include "_coder_net_predict_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[4] = {
      "789ce5534b4ec330147450416c0add700c20a565012b44512920a0280d20611452e755b5"
      "1adb91edf4b3e3069c806b7016161c06d236258d14a542a22c9845c6"
      "a3c97bf3fc9420e3ecd240086da0098ed6265c9cead29457d03cd2be91c131565161ae2e"
      "f65fa64c04d730d413c15d06b34a4f30ca5dae5ba300900425fc3e78",
      "63a7437d68510656525c458ad513d64c445674ae7581f4ac9021d955df13fa4931dbc753"
      "c67d0b39fb4823bd8ff47bff25effd877971ff7e4e5eec3fd88fb543"
      "6c2b900a570e2a55139f081232e05ae153aa1b611b5bad6d468914e003d152704a1466ae"
      "f6dd36768340450f27546d47015742828739e881903d85b917385c50",
      "054eb76c3a55b3d7289b8d9dafdafcbd6e2e78cfacffa888d6c7bc6b7c8cad65e53dbfb2"
      "b765e6c5f8abbc6146bf45bfd3ad8cbc52cadfbf18d43bf675f3fcae"
      "66efdd9251f5b87e7f9398a39993933707cad0bfddff13f8a56567",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 1624U, &nameCaptureInfo);
  return nameCaptureInfo;
}

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
                emlrtMxCreateString("net_predict"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "ResolvedFilePath",
                emlrtMxCreateString("C:\\Users\\39340\\Documents\\GitHub\\ST-"
                                    "microelectronics\\matlab\\apps\\app_usb_"
                                    "sensored\\networks\\net_predict.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739830.68927083339));
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

/* End of code generation (_coder_net_predict_info.c) */
